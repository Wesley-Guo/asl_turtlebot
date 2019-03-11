#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped
from asl_turtlebot.msg import DetectedObject, DetectedObjectList
import tf
import tf.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion
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
CROSSING_TIME = 60

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
        self.modeBeforeStop = None
        self.last_mode_printed = None
        self.trans_listener = tf.TransformListener()
        # command pose for controller
        self.pose_goal_publisher = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        # nav pose for controller
        self.nav_goal_publisher = rospy.Publisher('/cmd_nav', Pose2D, queue_size=10)
        # command vel (used for idling)
        self.cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.objects_dict = {}
        self.delivery_requests = []
        self.home_base = "elephant"
        self.home_base_dict = {}
        self.goal_list = []

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
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            pass
        self.mode = Mode.NAV

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
        for i in range(len(msg.objects)):
            add_to_dict(msg.objects[i], msg.ob_msgs[i])

    def add_to_dict(object_name, object_msg):
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

        pt = PoseStamped()
        pt.header.frame_id = '/camera'
        pt.header.stamp = rospy.get_rostime()
        pt.pose.position.x = dist*np.sin(theta_mid)
        pt.pose.position.y = 0
        pt.pose.position.z = dist*np.cos(theta_mid)
        quat = tf.transformations.quaternion_from_euler(0,theta_mid,0)
        pt.pose.orientation.x = quat[0]
        pt.pose.orientation.y = quat[1]
        pt.pose.orientation.z = quat[2]
        pt.pose.orientation.w = quat[3]

        object_map_pt = self.tf_listener.transformPose("/map", pt)

        euler_angles = tf.transformations.euler_from_quaternion(object_map_pt.pose.orientation)

        object_map_pose = Pose2D()
        object_map_pose.x = object_map_pt.pose.position.x
        object_map_pose.y = object_map_pose.pose.position.y
        object_map_pose.theta = euler_angles[2]

        if object_name in self.objects_dict:
            prevCount = self.objects_dict[object_name][1]
            prevPoint = self.objects_dict[object_name][0]
            newPointVec = [((object_map_pose.x+(prevPoint.x*prevCount))/(prevCount+1)), 
                            ((object_map_pose.y+(prevPoint.y*prevCount))/(prevCount+1)), 
                                ((object_map_pose.theta+(prevPoint.theta*prevCount))/(prevCount+1))]
            newPoint = Pose2D()
            newPoint.x = newPointVec[0]
            newPoint.y = newPointVec[1]
            newPoint.theta = newPointVec[2]
            self.objects_dict[object_name] = (newPoint, prevCount+ 1)
        else:
            self.objects_dict[object_name] = (object_map_pose, 1)
        


    #Adam edit - detector for food 1 - TODO: change to actual 
    # def pizza_detected_callback(self, msg):
    # 	dist = msg.distance
    # 	print("pizza distance")
    # 	print(dist)
    # 	theta_left = msg.thetaleft
    # 	theta_right = msg.thetaright
    # 	theta_mid = 0
    # 	if theta_left >= theta_right:
    # 		theta_mid = (theta_left+theta_right)/2
    # 	else:
    # 		theta_mid = (theta_left + (theta_right - 2*np.pi))/2

    # 	pt = PointStamped()
    #     pt.header.frame_id = '/camera'
    #     pt.header.stamp = rospy.get_rostime()
    #     pt.point.x = dist*np.sin(theta_mid)
    #     pt.point.y = 0
    #     pt.point.z = dist*np.cos(theta_mid)

    # 	pizza_map_pt = self.tf_listener.transformPoint("/map", pt)

    # 	if self.objects_dict["pizza"]:
    # 		prevPoint = self.objects_dict["pizza"]
    # 		newPointVec = [pizza_map_pt.point.x+prevPoint.point.x, pizza_map_pt.point.y + 
    # 							prevPoint.point.y, pizza_map_pt.point.z + prevPoint.point.z]/2
    # 		newPoint = PointStamped()
    # 		newPoint.header.frame_id = '/camera'
	   #      newPoint.header.stamp = rospy.get_rostime()
	   #      newPoint.point.x = newPointVec[0]
	   #      newPoint.point.y = newPointVec[1]
	   #      newPoint.point.z = newPointVec[2]
	   #      self.objects_dict["pizza"] = newPoint
	   #  else:
	   #  	self.objects_dict["pizza"] = pizza_map_pt


    	#todo - store TF frame in dict

    # def broccoli_detected_callback(self,msg):
    # 	dist = msg.distance
    # 	print("broccoli distance")
    # 	print(dist)
    # 	theta_left = msg.thetaleft
    # 	theta_right = msg.thetaright
    # 	theta_mid = 0
    # 	if theta_left >= theta_right:
    # 		theta_mid = (theta_left+theta_right)/2
    # 	else:
    # 		theta_mid = (theta_left + (theta_right - 2*np.pi))/2

    # 	pt = PointStamped()
    #     pt.header.frame_id = '/camera'
    #     pt.header.stamp = rospy.get_rostime()
    #     pt.point.x = dist*np.sin(theta_mid)
    #     pt.point.y = 0
    #     pt.point.z = dist*np.cos(theta_mid)

    # 	broccoli_map_pt = self.tf_listener.transformPoint("/map", pt)

    # 	if self.objects_dict["broccoli"]:
    # 		prevPoint = self.objects_dict["broccoli"]
    # 		newPointVec = [broccoli_map_pt.point.x+prevPoint.point.x, broccoli_map_pt.point.y + 
    # 							prevPoint.point.y, broccoli_map_pt.point.z + prevPoint.point.z]/2
    # 		newPoint = PointStamped()
    # 		newPoint.header.frame_id = '/camera'
	   #      newPoint.header.stamp = rospy.get_rostime()
	   #      newPoint.point.x = newPointVec[0]
	   #      newPoint.point.y = newPointVec[1]
	   #      newPoint.point.z = newPointVec[2]
	   #      self.objects_dict["broccoli"] = newPoint
	   #  else:
	   #  	self.objects_dict["broccoli"] = broccoli_map_pt



    # 	#todo store TF frame in dict

    # def elephant_detected_callback(self, msg):

    # 	dist = msg.distance
    # 	print("elephant distance")
    # 	print(dist)
    # 	theta_left = msg.thetaleft
    # 	theta_right = msg.thetaright
    # 	theta_mid = 0
    # 	if theta_left >= theta_right:
    # 		theta_mid = (theta_left+theta_right)/2
    # 	else:
    # 		theta_mid = (theta_left + (theta_right - 2*np.pi))/2

    # 	pt = PointStamped()
    #     pt.header.frame_id = '/camera'
    #     pt.header.stamp = rospy.get_rostime()
    #     pt.point.x = dist*np.sin(theta_mid)
    #     pt.point.y = 0
    #     pt.point.z = dist*np.cos(theta_mid)

    # 	elephant_map_pt = self.tf_listener.transformPoint("/map", pt)

    # 	if self.objects_dict["elephant"]:
    # 		prevPoint = self.objects_dict["elephant"]
    # 		newPointVec = [elephant_map_pt.point.x+prevPoint.point.x, elephant_map_pt.point.y + 
    # 							prevPoint.point.y, elephant_map_pt.point.z + prevPoint.point.z]/2
    # 		newPoint = PointStamped()
    # 		newPoint.header.frame_id = '/camera'
	   #      newPoint.header.stamp = rospy.get_rostime()
	   #      newPoint.point.x = newPointVec[0]
	   #      newPoint.point.y = newPointVec[1]
	   #      newPoint.point.z = newPointVec[2]
	   #      self.objects_dict["elephant"] = newPoint
	   #  else:
	   #  	self.objects_dict["elephant"] = elephant_map_pt


    # 	#todo store TF frame in dict
    # def giraffe_detected_callback(self,msg):
    # 	dist = msg.distance
    # 	print("giraffe distance")
    # 	print(dist)
    # 	theta_left = msg.thetaleft
    # 	theta_right = msg.thetaright
    # 	theta_mid = 0
    # 	if theta_left >= theta_right:
    # 		theta_mid = (theta_left+theta_right)/2
    # 	else:
    # 		theta_mid = (theta_left + (theta_right - 2*np.pi))/2

    # 	pt = PointStamped()
    #     pt.header.frame_id = '/camera'
    #     pt.header.stamp = rospy.get_rostime()
    #     pt.point.x = dist*np.sin(theta_mid)
    #     pt.point.y = 0
    #     pt.point.z = dist*np.cos(theta_mid)

    # 	giraffe_map_pt = self.tf_listener.transformPoint("/map", pt)

    # 	if self.objects_dict["giraffe"]:
    # 		prevPoint = self.objects_dict["giraffe"]
    # 		newPointVec = [giraffe_map_pt.point.x+prevPoint.point.x, giraffe_map_pt.point.y + 
    # 							prevPoint.point.y, giraffe_map_pt.point.z + prevPoint.point.z]/2
    # 		newPoint = PointStamped()
    # 		newPoint.header.frame_id = '/camera'
	   #      newPoint.header.stamp = rospy.get_rostime()
	   #      newPoint.point.x = newPointVec[0]
	   #      newPoint.point.y = newPointVec[1]
	   #      newPoint.point.z = newPointVec[2]
	   #      self.objects_dict["giraffe"] = newPoint
	   #  else:
	   #  	self.objects_dict["giraffe"] = giraffe_map_pt
    	#todo store TF frame in dict


    def delivery_request_callback(self,msg):
        label_list = msg.split(',')
        for label in label_list:
            self.goal_list.append(self.objects_dict[label])
        #reorder list to make optimal?
        self.goal_list.append(self.objects_dict[self.home_base])
        self.Mode = Mode.NAV

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
                pass

        # logs the current mode
        if not(self.last_mode_printed == self.mode):
            rospy.loginfo("Current Mode: %s", self.mode)
            self.last_mode_printed = self.mode

        # checks wich mode it is in and acts accordingly
        if self.mode == Mode.IDLE:
            # send zero velocity
            self.stay_idle()
            if

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
                self.mode = self.
            else:
                self.nav_to_pose()

        elif self.mode == Mode.NAV:
            while self.goal_list:
                goal = goal_list[0]
                self.x_g = goal.x
                self.y_g = goal.y
                self.theta_g = goal.theta
                self.nav_to_pose()
                if self.close_to(self.x_g,self.y_g,self.theta_g):
                    del self.goal_list[0]
            self.Mode = Mode.IDLE 

        elif self.mode == Mode.EXPLORE:
            self.init_explore_start()
            if self.has_explored():
                self.Mode = Mode.IDLE
            else:
                pass

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
