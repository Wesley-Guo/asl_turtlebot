#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject, DetectedObjectList
import tf
import tf.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
import math
from enum import Enum
import numpy as np

# if sim is True/using gazebo, therefore want to subscribe to /gazebo/model_states\
# otherwise, they will use a TF lookup (hw2+)
use_gazebo = rospy.get_param("sim")

# if using gmapping, you will have a map frame. otherwise it will be odom frame
mapping = rospy.get_param("map")


# threshold at which we consider the robot at a location
POS_EPS = .1
THETA_EPS = .3

# time to stop at a stop sign
STOP_TIME = 3

# minimum distance from a stop sign to obey it
STOP_MIN_DIST = .5

# time taken to cross an intersection
CROSSING_TIME = 3

# time to autonomously explore
EXPLORE_TIME = 120

# state machine modes, not all implemented
class Mode(Enum):
    IDLE = 1
    POSE = 2
    STOP = 3
    CROSS = 4
    NAV = 5
    MANUAL = 6
    EXPLORE = 7

print "supervisor settings:\n"
print "use_gazebo = %s\n" % use_gazebo
print "mapping = %s\n" % mapping

class Supervisor:

    def __init__(self):
        rospy.init_node('turtlebot_supervisor', anonymous=True)
        # initialize variables
        self.x = 0
        self.y = 0
        self.theta = 0
        self.mode = Mode.IDLE
        self.x_g = 0
        self.y_g = 0
        self.theta_g = 0
        self.modeBeforeStop = None
        self.last_mode_printed = None
        self.trans_listener = tf.TransformListener()
        # command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        # nav pose for controller
        self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        # command vel (used for idling)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_publisher = rospy.Publisher('/marker_array', MarkerArray, queue_size=10)
        self.objects_dict = {}
        self.delivery_requests = []
        self.home_base = "bottle"
        self.home_base_dict = {}
        self.goal_list = []
        self.rviz_goal = False
        self.explore_start = rospy.get_rostime()
        self.marker_array = MarkerArray()
        self.acceptable_objects = ['pizza','orange','sandwich','elephant', 'broccoli', 'chair', 
                                    'cake','apple','hot dog','banana','donut','pizza','carrot']

        if use_gazebo:
            self.camera_frame_id = '/base_camera'
        else:
            self.camera_frame_id = '/camera'

        # subscribers
        # stop sign detector
        rospy.Subscriber('/detector/stop_sign', DetectedObject, self.stop_sign_detected_callback)
        #food/house detector
        # rospy.Subscriber('/detector/pizza', DetectedObject, self.pizza_detected_callback)
        # rospy.Subscriber('/detector/broccoli', DetectedObject, self.broccoli_detected_callback)
        # rospy.Subscriber('/detector/elephant', DetectedObject, self.elephant_detected_callback)
        # rospy.Subscriber('/detector/giraffe', DetectedObject, self.giraffe_detected_callback)
        # rospy.Subscriber('/delivery/getFood', Delivery, self.delivery_callback)
        rospy.Subscriber('/detector/objects', DetectedObjectList, self.object_list_callback)
        # high-level navigation pose
        rospy.Subscriber('/nav_pose', Pose2D, self.nav_pose_callback)
        # if using gazebo, we have access to perfect state
        if use_gazebo:
            rospy.Subscriber('/gazebo/model_states', ModelStates, self.gazebo_callback)
        # we can subscribe to nav goal click
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.rviz_goal_callback)
        # deliveries subscriber
        rospy.Subscriber('/delivery_request', String, self.delivery_request_callback)
        # stat exploration from keyboard command subscriber
        # rospy.Subscriber('/keyboard_commands', String, self.keyboard_callback)
        # Subscriber for keyboard command to move into explore mode
        rospy.Subscriber('/start_explore', String, self.idle_to_explore_callback)

    def idle_to_explore_callback(self, msg):
        self.init_explore_start()

    def gazebo_callback(self, msg):
        pose = msg.pose[msg.name.index("turtlebot3_burger")]
        twist = msg.twist[msg.name.index("turtlebot3_burger")]
        self.x = pose.position.x
        self.y = pose.position.y
        quaternion = (
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.theta = euler[2]

    def rviz_goal_callback(self, msg):
        """ callback for a pose goal sent through rviz """
        origin_frame = "/map" if mapping else "/odom"
        print("rviz command received!")
        try:
            # Only respond to RVIZ clicks if we are in explore mode
            if self.mode == Mode.EXPLORE: 
                print("sending new goal")
                nav_pose_origin = self.trans_listener.transformPose(origin_frame, msg)
                self.x_g = nav_pose_origin.pose.position.x
                self.y_g = nav_pose_origin.pose.position.y
                quaternion = (
                        nav_pose_origin.pose.orientation.x,
                        nav_pose_origin.pose.orientation.y,
                        nav_pose_origin.pose.orientation.z,
                        nav_pose_origin.pose.orientation.w)
                euler = tf.transformations.euler_from_quaternion(quaternion)
                self.theta_g = euler[2]

                # send goal to navigator node.
                self.rviz_goal = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print('tf error in rviz_goal_callback')
            pass


    def nav_pose_callback(self, msg):
        self.x_g = msg.x
        self.y_g = msg.y
        self.theta_g = msg.theta
        self.mode = Mode.NAV

    def stop_sign_detected_callback(self, msg):
        """ callback for when the detector has found a stop sign. Note that
        a distance of 0 can mean that the lidar did not pickup the stop sign at all """

        # distance of the stop sign
        dist = msg.distance
        print("stop sign distance")
        print(dist)
        # if close enough and in nav mode, stop
        if dist > 0 and dist < STOP_MIN_DIST and self.mode == Mode.NAV or self.mode == Mode.EXPLORE:
            self.init_stop_sign()

    def object_list_callback(self, msg):
        if self.mode == Mode.EXPLORE:
            for i in range(len(msg.objects)):
                if msg.objects[i] in  self.acceptable_objects and msg.ob_msgs[i].confidence > 0.5:
                    self.add_to_dict(msg.objects[i], msg.ob_msgs[i])
                    rospy.loginfo(msg.objects[i])
        # print("added all to dict")
        # print(self.objects_dict)

    def idle_to_explore_callback(self, msg):
        self.init_explore_start()


    def add_to_dict(self, object_name, object_msg):
        # pass
        dist = object_msg.distance
        print( object_name + " distance")
        print(dist)
        theta_left = object_msg.thetaleft
        theta_right = object_msg.thetaright
        theta_mid = 0
        if theta_left >= theta_right:
            theta_mid = (theta_left+theta_right)/2
        else:
            theta_mid = (theta_left + (theta_right - 2*np.pi))/2

        print("theta_right:" + str(theta_right))
        print("theta_left:" + str(theta_left))
        print("theta_mid:" + str(theta_mid))
        print("dist:" + str(dist))
        pt = PoseStamped()
        pt.header.frame_id = self.camera_frame_id
        pt.header.stamp = rospy.Time(0)
        pt.pose.position.x = dist*np.sin(theta_mid)
        pt.pose.position.y = 0
        pt.pose.position.z = dist*np.cos(theta_mid)
        quat = tf.transformations.quaternion_from_euler(0,theta_mid,0)
        pt.pose.orientation.x = quat[0]
        pt.pose.orientation.y = quat[1]
        pt.pose.orientation.z = quat[2]
        pt.pose.orientation.w = quat[3]

        self.trans_listener.waitForTransform(pt.header.frame_id, "/map", rospy.Time(0), rospy.Duration(1.0))
        object_map_pt = self.trans_listener.transformPose("/map", pt)

        euler_angles = tf.transformations.euler_from_quaternion([object_map_pt.pose.orientation.x,object_map_pt.pose.orientation.y,
                                                                    object_map_pt.pose.orientation.z, object_map_pt.pose.orientation.w])

        object_map_pose = Pose2D()
        object_map_pose.x = object_map_pt.pose.position.x
        object_map_pose.y = object_map_pt.pose.position.y
        object_map_pose.theta = euler_angles[2]

        if object_name in self.objects_dict:
            sumWeights = self.objects_dict[object_name][4]
            sumCoords_theta = self.objects_dict[object_name][3]
            sumCoords_y = self.objects_dict[object_name][2]
            sumCoords_x = self.objects_dict[object_name][1]
            prevPoint = self.objects_dict[object_name][0]
            # don't average if the distance is nota number. 
            if not(np.isnan(object_map_pose.x) and np.isnan(object_map_pose.y)):
                newPointVec = [(((object_map_pose.x*np.cos(theta_mid))+ sumCoords_x)/sumWeights), 
                                (((object_map_pose.y*np.cos(theta_mid))+ sumCoords_y)/sumWeights), 
                                    (((object_map_pose.theta*np.cos(theta_mid))+ sumCoords_theta)/sumWeights)]
                newPoint = Pose2D()
                newPoint.x = newPointVec[0]
                newPoint.y = newPointVec[1]
                newPoint.theta = newPointVec[2]
                self.objects_dict[object_name] = (newPoint, sumCoords_x + (object_map_pose.x*np.cos(theta_mid)),
                                                    sumCoords_y + (object_map_pose.y*np.cos(theta_mid)),
                                                    sumCoords_theta + (object_map_pose.theta*np.cos(theta_mid)),
                                                     sumWeights + np.cos(theta_mid))
        else:
            self.objects_dict[object_name] = (object_map_pose, (object_map_pose.x*np.cos(theta_mid)),
                                                (object_map_pose.y*np.cos(theta_mid)),
                                                (object_map_pose.theta*np.cos(theta_mid)),
                                                np.cos(theta_mid))

        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.objects_dict[object_name][0].x
        marker.pose.position.y = self.objects_dict[object_name][0].y
        marker.pose.position.z = 0
        self.marker_array.markers.append(marker)

        self.marker_publisher.publish(self.marker_array)
    

    def delivery_request_callback(self,msg):
        label_list = msg.data.split(',')
        for label in label_list:
            print "label:", label
            self.goal_list.append(self.objects_dict[label][0])
        #reorder list to make optimal?
        self.goal_list.append(self.objects_dict[self.home_base])
        # print "goal list:", self.goal_list
        self.mode = Mode.NAV

    def go_to_pose(self):
        """ sends the current desired pose to the pose controller """

        pose_g_msg = Pose2D()
        pose_g_msg.x = self.x_g
        pose_g_msg.y = self.y_g
        pose_g_msg.theta = self.theta_g

        self.pose_goal_publisher.publish(pose_g_msg)

    def nav_to_pose(self):
        """ sends the current desired pose to the naviagtor """

        nav_g_msg = Pose2D()
        nav_g_msg.x = self.x_g
        nav_g_msg.y = self.y_g
        nav_g_msg.theta = self.theta_g

        self.nav_goal_publisher.publish(nav_g_msg)

    def stay_idle(self):
        """ sends zero velocity to stay put """

        vel_g_msg = Twist()
        self.cmd_vel_publisher.publish(vel_g_msg)

    def close_to(self,x,y,theta):
        """ checks if the robot is at a pose within some threshold """

        return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS and abs(theta-self.theta)<THETA_EPS)

    def init_stop_sign(self):
        """ initiates a stop sign maneuver """

        self.stop_sign_start = rospy.get_rostime()
        self.modeBeforeStop = self.mode
        self.mode = Mode.STOP

    def init_explore_start(self):
        self.explore_start = rospy.get_rostime()
        self.mode = Mode.EXPLORE
        #implement autonomous exploration
        #self.nav_to_pose()

    def has_explored(self):
        """ checks if exploration is over """
        # print "Check explore time:"
        # print(rospy.get_rostime()-self.explore_start)
        # print(rospy.Duration.from_sec(EXPLORE_TIME))
        # print (self.mode == Mode.EXPLORE and (rospy.get_rostime()-self.explore_start)>rospy.Duration.from_sec(EXPLORE_TIME))
        return (self.mode == Mode.EXPLORE and (rospy.get_rostime()-self.explore_start)>rospy.Duration.from_sec(EXPLORE_TIME))

    def has_stopped(self):
        """ checks if stop sign maneuver is over """

        return (self.mode == Mode.STOP and (rospy.get_rostime()-self.stop_sign_start)>rospy.Duration.from_sec(STOP_TIME))

    def init_crossing(self):
        """ initiates an intersection crossing maneuver """

        self.cross_start = rospy.get_rostime()
        self.mode = Mode.CROSS

    def has_crossed(self):
        """ checks if crossing maneuver is over """
        return (self.mode == Mode.CROSS and (rospy.get_rostime()-self.cross_start)>rospy.Duration.from_sec(CROSSING_TIME))

    def loop(self):
        """ the main loop of the robot. At each iteration, depending on its
        mode (i.e. the finite state machine's state), if takes appropriate
        actions. This function shouldn't return anything """

        if not use_gazebo:
            try:
                origin_frame = "/map" if mapping else "/odom"
                (translation,rotation) = self.trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
                self.x = translation[0]
                self.y = translation[1]
                euler = tf.transformations.euler_from_quaternion(rotation)
                self.theta = euler[2]
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print('tf error occured when looking up transform from /map to /base_footprint')
                pass

        # logs the current mode
        if not(self.last_mode_printed == self.mode):
            rospy.loginfo("Current Mode: %s", self.mode)
            self.last_mode_printed = self.mode

        # checks wich mode it is in and acts accordingly
        if self.mode == Mode.IDLE:
            # send zero velocity
            self.stay_idle()
            
        elif self.mode == Mode.POSE:
            # moving towards a desired pose
            if self.close_to(self.x_g,self.y_g,self.theta_g):
                self.mode = Mode.IDLE
            else:
                self.go_to_pose()

        elif self.mode == Mode.STOP:
            # at a stop sign
            if self.has_stopped():
                self.init_crossing()
            else:
                self.stay_idle()

        elif self.mode == Mode.CROSS:
            # crossing an intersection
            if self.has_crossed():
                self.mode = self.modeBeforeStop
            else:
                self.nav_to_pose()

        elif self.mode == Mode.NAV:
            while self.goal_list:
                goal = self.goal_list[0]
                # print "Goal:", goal
                self.x_g = goal.x
                self.y_g = goal.y
                self.theta_g = goal.theta
                self.nav_to_pose()
                if self.close_to(self.x_g,self.y_g,self.theta_g):
                    del self.goal_list[0]
            self.mode = Mode.IDLE 

        elif self.mode == Mode.EXPLORE:
            # self.init_explore_start()
            if self.has_explored():
                print "have explored"
                self.mode = Mode.IDLE
            elif self.rviz_goal:
                self.nav_to_pose()
                # print(self.close_to(self.x_g,self.y_g,self.theta_g))
                if self.close_to(self.x_g,self.y_g,self.theta_g):
                    self.rviz_goal = False

        else:
            raise Exception('This mode is not supported: %s'
                % str(self.mode))

    def run(self):
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

if __name__ == '__main__':
    sup = Supervisor()
    sup.run()
