#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Twist, PoseArray, Pose2D, PoseStamped, PointStamped
from std_msgs.msg import Float32MultiArray, String
import tf
import numpy as np
from numpy import linalg
from utils import wrapToPi
from astar import AStar
from grids import StochOccupancyGrid2D
import scipy.interpolate
import matplotlib.pyplot as plt
import copy
import skimage
from skimage.morphology import square


# threshold at which navigator switches
# from trajectory to pose control
END_POS_THRESH = .2

# threshold to be far enough into the plan
# to recompute it
START_POS_THRESH = .2

# thereshold in theta to start moving forward when path following
THETA_START_THRESH = 0.09
# P gain on orientation before start
THETA_START_P = 1

# maximum velocity
V_MAX = .1

# maximim angular velocity
W_MAX = .2

# desired crusing velocity
V_DES = 0.12

# gains of the path follower
KPX = .5
KPY = .5
KDX = 1.5
KDY = 1.5

# smoothing condition (see splrep documentation)
SMOOTH = .01

PUDDLE_TOLERANCE = 0.1
PUDDLE_SIZE = 5 #MUST BE ODD NUMBER
WALL_DILATION_SIZE = 2

POS_EPS = .1
THETA_EPS = .3

class Navigator:

    def __init__(self):
        rospy.init_node('turtlebot_navigator', anonymous=True)

        # current state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.starting = False

        # goal stateoccupancy_updated
        self.x_g = 0.0
        self.y_g = 0.0
        self.theta_g = 0.0

        self.x_way = 0.0
        self.y_way = 0.0
        self.theta_way = 0.0

        self.current_plan = []

        # map parameters
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0
        self.map_origin = [0,0]
        self.map_probs = []
        self.occupancy = None
        self.astar_occupancy = None  #This is the occupancy grid that actually gets passed to astar
        self.occupancy_updated = False

        # plan parameters
        self.plan_resolution =  0.05
        self.plan_horizon = 15

        # variables for the controller
        self.V_prev = 0
        self.V_prev_t = rospy.get_rostime()
        
        self.sequence = 0
        
        self.orientation = 1
        
        #dictionary for list of puddles
        self.puddle_list = []

        self.nav_path_pub = rospy.Publisher('/cmd_path', Path, queue_size=10)
        self.nav_pose_pub = rospy.Publisher('/cmd_pose', Pose2D, queue_size=10)
        self.nav_pathsp_pub = rospy.Publisher('/cmd_path_sp', PoseStamped, queue_size=10)
        self.nav_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.map_mod = rospy.Publisher('/map_mod', OccupancyGrid, queue_size=10)

        self.trans_listener = tf.TransformListener()

        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/map_metadata', MapMetaData, self.map_md_callback)
        rospy.Subscriber('/cmd_nav', Pose2D, self.cmd_nav_callback)
        rospy.Subscriber('puddle_world', PointStamped, self.puddle_callback)
        rospy.Subscriber('/clicked_point', PointStamped, self.manual_puddle_callback)
        # rospy.Subscriber('/sup_request', Point, self.is_free_callback)

    # def is_free_callback(self, msg):
    #     x_point = msg.x
    #     y_point = msg.y
    #     z_point = msg.z

    def close_to(self,x,y,theta):
        """ checks if the robot is at a pose within some threshold """

        return (abs(x-self.x)<POS_EPS and abs(y-self.y)<POS_EPS and abs(theta-self.theta)<THETA_EPS)

    def cmd_nav_callback(self, data):
        self.x_g = data.x
        self.y_g = data.y
        self.theta_g = data.theta
        self.run_navigator()

    def map_md_callback(self, msg):
        self.map_width = msg.width
        self.map_height = msg.height
        self.map_resolution = msg.resolution
        self.map_origin = (msg.origin.position.x,msg.origin.position.y)

    def map_callback(self,msg):
        print("in map_callback method")
        self.map_probs = msg.data
        self.orientation = msg.info.origin.orientation.w
        if self.map_width>0 and self.map_height>0 and len(self.map_probs)>0:
            self.occupancy = StochOccupancyGrid2D(self.map_resolution,
                                                  self.map_width,
                                                  self.map_height,
                                                  self.map_origin[0],
                                                  self.map_origin[1],
                                                  6,
                                                  self.map_probs)
            self.occupancy_updated = True
            
            self.publish_map_mod()
    
    def puddle_callback(self,msg):
        puddle_repeated = False
        if self.puddle_list:
            for puddle in self.puddle_list:
                puddle_x_difference = puddle[0] - msg.point.x
                puddle_y_difference = puddle[1] - msg.point.y
                if puddle_x_difference < 0:
                    puddle_x_difference *= -1
                if puddle_y_difference < 0:
                    puddle_y_difference *= -1
                if (puddle_x_difference < PUDDLE_TOLERANCE) and (puddle_y_difference < PUDDLE_TOLERANCE):
                    puddle_repeated = True
                    break
        if not puddle_repeated:
            self.puddle_list.append([msg.point.x,msg.point.y])
            self.publish_map_mod()
    
    def manual_puddle_callback(self,msg):
        self.puddle_callback(msg)
            
    def publish_map_mod(self):
        self.astar_occupancy = copy.copy(self.occupancy)

        #TARIQ EDIT: 
        # replace all -1's with 69


        #self.conflate_obstacles()
        if self.puddle_list:
            self.puddle_to_occupancy()
        msg2send = OccupancyGrid()
        msg2send.header.seq = self.sequence
        msg2send.header.stamp = rospy.Time.now()
        msg2send.header.frame_id = "/map"
        msg2send.info.resolution = self.astar_occupancy.resolution
        msg2send.info.width = self.astar_occupancy.width
        msg2send.info.height = self.astar_occupancy.height
        msg2send.info.origin.position.x = self.astar_occupancy.origin_x
        msg2send.info.origin.position.y = self.astar_occupancy.origin_y
        msg2send.info.origin.orientation.w = self.orientation
        msg2send.data = self.astar_occupancy.probs
        self.map_mod.publish(msg2send)
        self.sequence += 1
    
    def puddle_to_occupancy(self):
        for puddle in self.puddle_list:
            raw_x_occ = (puddle[0]-self.astar_occupancy.origin_x)
            raw_y_occ = (puddle[1]-self.astar_occupancy.origin_y)
            (puddle_row,puddle_col) = self.astar_occupancy.snap_to_grid([raw_x_occ,raw_y_occ])
            puddle_rowid = puddle_row/self.astar_occupancy.resolution
            puddle_colid = puddle_col/self.astar_occupancy.resolution
            index = self.astar_occupancy.width*puddle_rowid+puddle_colid
            #self.astar_occupancy.probs[index] = 93
            self.fill_in_puddle(puddle_rowid,puddle_colid)


    def fill_in_puddle(self,puddle_rowid, puddle_colid):
        #Use either manual dilation or SKImage
        #Possibly use DBScan for clustering
        occupancy = list(copy.copy(self.astar_occupancy.probs))
        for i in range(PUDDLE_SIZE):
            for j in range(PUDDLE_SIZE):
                #TODO edge case if center of puddle is at edge of map
                index = self.astar_occupancy.width*(puddle_colid-(PUDDLE_SIZE/2)+i)+(puddle_rowid-(PUDDLE_SIZE/2)+j)
                if (not index < 0) and (not index >= len(self.astar_occupancy.probs)):
                    occupancy[int(index)] = 100
                    self.astar_occupancy.probs = occupancy

    def conflate_obstacles(self):
        #TODO
        #Conflate obstacles in occupancy grid
        occupancy = np.array(copy.copy(self.astar_occupancy.probs))
        unknown_idx = np.where(occupancy==-1,-1,0)
        occ_grid_to_dilate = np.reshape(np.where(occupancy < 50,0,1),(self.astar_occupancy.height,self.astar_occupancy.width))
        dilated_grid_raw = skimage.morphology.dilation(occ_grid_to_dilate, square(WALL_DILATION_SIZE))
        dilated_grid_final = np.ndarray.flatten(np.where(dilated_grid_raw == 1,100,0))+unknown_idx
        self.astar_occupancy.probs = dilated_grid_final

    def close_to_end_location(self):
        return (abs(self.x-self.x_g)<END_POS_THRESH and abs(self.y-self.y_g)<END_POS_THRESH)

    def snap_to_grid(self, x):
        return (self.plan_resolution*round(x[0]/self.plan_resolution), self.plan_resolution*round(x[1]/self.plan_resolution))

    def close_to_start_location(self):
        if len(self.current_plan)>0:
            snapped_current = self.snap_to_grid([self.x, self.y])
            snapped_start = self.snap_to_grid(self.current_plan_start_loc)
            return (abs(snapped_current[0]-snapped_start[0])<START_POS_THRESH and abs(snapped_current[1]-snapped_start[1])<START_POS_THRESH)
        return False

    def run_navigator(self):
        """ computes a path from current state to goal state using A* and sends it to the path controller """

        # makes sure we have a location
        try:
            (translation,rotation) = self.trans_listener.lookupTransform('/map', '/base_footprint', rospy.Time(0))
            self.x = translation[0]
            self.y = translation[1]
            euler = tf.transformations.euler_from_quaternion(rotation)
            self.theta = euler[2]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            self.current_plan = []
            return

        # makes sure we have a map
        if not self.occupancy:
            self.current_plan = []
            return

        # if close to the goal, use the pose_controller instead
        if self.close_to_end_location():
            rospy.loginfo("Navigator: Close to nav goal using pose controller")
            pose_g_msg = Pose2D()
            pose_g_msg.x = self.x_g
            pose_g_msg.y = self.y_g
            pose_g_msg.theta = self.theta_g
            self.nav_pose_pub.publish(pose_g_msg)
            self.current_plan = []
            self.V_prev = 0
            return

        # if there is no plan, we are far from the start of the plan,
        # or the occupancy grid has been updated, update the current plan
        if len(self.current_plan)==0 or not(self.close_to_start_location()) or self.occupancy_updated:

            # use A* to compute new plan
            state_min = self.snap_to_grid((-self.plan_horizon, -self.plan_horizon))
            state_max = self.snap_to_grid((self.plan_horizon, self.plan_horizon))
            x_init = self.snap_to_grid((self.x, self.y))
            x_goal = self.snap_to_grid((self.x_g, self.y_g))

            # check if the init is free (in case we hit a wall)
            
            #self.astar_occupancy = copy.copy(self.occupancy)
            #self.conflate_obstacles()
            #--- Add known puddles to occupancy grid ---
            #if self.puddle_list:
            #    self.puddle_to_occupancy()
            print("trying to solve with a star")
            problem = AStar(state_min,state_max,x_init,x_goal,self.astar_occupancy,self.plan_resolution)
            rospy.logwarn("Is x_init in collision-free?")
            rospy.logwarn(problem.is_free(x_init))


            rospy.loginfo("Navigator: Computing navigation plan")
            if problem.solve():
                self.starting = True
                # if len(problem.path) > 3:
                    # cubic spline interpolation requires 4 points
                self.current_plan = problem.path
                self.current_plan_start_time = rospy.get_rostime()
                self.current_plan_start_loc = [self.x, self.y]
                self.occupancy_updated = False
                print("sel current plan 0")
                print(self.current_plan[0])
                rospy.logwarn(self.current_plan)
                self.x_way = self.current_plan[0][0]
                self.y_way = self.current_plan[0][1]
                self.theta_way = 0

                # publish plan for visualization
                path_msg = Path()
                path_msg.header.frame_id = 'map'
                for state in self.current_plan:
                    pose_st = PoseStamped()
                    pose_st.pose.position.x = state[0]
                    pose_st.pose.position.y = state[1]
                    pose_st.pose.orientation.w = 1
                    pose_st.header.frame_id = 'map'
                    path_msg.poses.append(pose_st)
                self.nav_path_pub.publish(path_msg)

            if self.current_plan:
                print("publishing plan to pose controller")
                if self.starting:
                    del self.current_plan[0]
                    self.starting = False
                goal = self.current_plan[0]
                waypt = Pose2D()
                waypt.x = self.x_way
                waypt.y = self.y_way
                if len(self.current_plan) == 1:
                    waypt.theta = self.theta_g
                else:
                    waypt.theta = self.theta_way

                self.nav_pose_pub.publish(waypt)

                # print "Goal:", goal
                
                if self.close_to(self.x_way,self.y_way, self.theta_way):
                    del self.current_plan[0]



                    # path_t = [0]
                    # path_x = [self.current_plan[0][0]]
                    # path_y = [self.current_plan[0][1]]
                    # for i in range(len(self.current_plan)-1):
                    #     dx = self.current_plan[i+1][0]-self.current_plan[i][0]
                    #     dy = self.current_plan[i+1][1]-self.current_plan[i][1]
                    #     path_t.append(path_t[i] + np.sqrt(dx**2 + dy**2) / V_DES)
                    #     path_x.append(self.current_plan[i+1][0])
                    #     path_y.append(self.current_plan[i+1][1])

                    # # interpolate the path with cubic spline
                    # self.path_x_spline = scipy.interpolate.splrep(path_t, path_x, k=3, s=SMOOTH)
                    # self.path_y_spline = scipy.interpolate.splrep(path_t, path_y, k=3, s=SMOOTH)
                    # self.path_tf = path_t[-1]

                    # # to inspect the interpolation and smoothing
                    # # t_test = np.linspace(path_t[0],path_t[-1],1000)
                    # # plt.plot(path_t,path_x,'ro')
                    # # plt.plot(t_test,scipy.interpolate.splev(t_test,self.path_x_spline,der=0))
                    # # plt.plot(path_t,path_y,'bo')
                    # # plt.plot(t_test,scipy.interpolate.splev(t_test,self.path_y_spline,der=0))
                    # # plt.show()
                # else:
                #     rospy.logwarn("Navigator: Path too short, not updating")
            else:
                rospy.logwarn("Navigator: Could not find path")
                self.current_plan = []
        # publish current desired x and y for visualization only
            # pathsp_msg = PoseStamped()
            # pathsp_msg.header.frame_id = 'map'
            # pathsp_msg.pose.position.x = x_d
            # pathsp_msg.pose.position.y = y_d
            # theta_d = np.arctan2(yd_d,xd_d)
            # quat_d = tf.transformations.quaternion_from_euler(0, 0, theta_d)
            # pathsp_msg.pose.orientation.x = quat_d[0]
            # pathsp_msg.pose.orientation.y = quat_d[1]
            # pathsp_msg.pose.orientation.z = quat_d[2]
            # pathsp_msg.pose.orientation.w = quat_d[3]
            # self.nav_pathsp_pub.publish(pathsp_msg)

        # if we have a path, execute it (we need at least 3 points for this controller)
        # if len(self.current_plan) > 3:

        #     # if currently not moving, first line up with the plan
        #     if self.V_prev == 0:
        #         theta_init = np.arctan2(self.current_plan[1][1]-self.current_plan[0][1],self.current_plan[1][0]-self.current_plan[0][0])
        #         theta_err = theta_init-self.theta
        #         if abs(theta_err)>THETA_START_THRESH:
        #             cmd_msg = Twist()
        #             cmd_msg.linear.x = 0
        #             cmd_msg.angular.z = THETA_START_P * theta_err
        #             self.nav_vel_pub.publish(cmd_msg)
        #             return

        #     # compute the "current" time along the path execution
        #     t = (rospy.get_rostime()-self.current_plan_start_time).to_sec()
        #     t = max(0.0, t)
        #     t = min(t, self.path_tf)

        #     x_d = scipy.interpolate.splev(t, self.path_x_spline, der=0)
        #     y_d = scipy.interpolate.splev(t, self.path_y_spline, der=0)
        #     xd_d = scipy.interpolate.splev(t, self.path_x_spline, der=1)
        #     yd_d = scipy.interpolate.splev(t, self.path_y_spline, der=1)
        #     xdd_d = scipy.interpolate.splev(t, self.path_x_spline, der=2)
        #     ydd_d = scipy.interpolate.splev(t, self.path_y_spline, der=2)

            # publish current desired x and y for visualization only
        #     pathsp_msg = PoseStamped()
        #     pathsp_msg.header.frame_id = 'map'
        #     pathsp_msg.pose.position.x = x_d
        #     pathsp_msg.pose.position.y = y_d
        #     theta_d = np.arctan2(yd_d,xd_d)
        #     quat_d = tf.transformations.quaternion_from_euler(0, 0, theta_d)
        #     pathsp_msg.pose.orientation.x = quat_d[0]
        #     pathsp_msg.pose.orientation.y = quat_d[1]
        #     pathsp_msg.pose.orientation.z = quat_d[2]
        #     pathsp_msg.pose.orientation.w = quat_d[3]
        #     self.nav_pathsp_pub.publish(pathsp_msg)

        #     if self.V_prev <= 0.0001:
        #         self.V_prev = linalg.norm([xd_d, yd_d])

        #     dt = (rospy.get_rostime()-self.V_prev_t).to_sec()

        #     xd = self.V_prev*np.cos(self.theta)
        #     yd = self.V_prev*np.sin(self.theta)

        #     u = np.array([xdd_d + KPX*(x_d-self.x) + KDX*(xd_d-xd),
        #                   ydd_d + KPY*(y_d-self.y) + KDY*(yd_d-yd)])
        #     J = np.array([[np.cos(self.theta), -self.V_prev*np.sin(self.theta)],
        #                   [np.sin(self.theta), self.V_prev*np.cos(self.theta)]])
        #     a, om = linalg.solve(J, u)
        #     V = self.V_prev + a*dt

        #     # apply saturation limits
        #     cmd_x_dot = np.sign(V)*min(V_MAX, np.abs(V))
        #     cmd_theta_dot = np.sign(om)*min(W_MAX, np.abs(om))
        # elif len(self.current_plan) > 0:
        #     # using the pose controller for paths too short
        #     # just send the next point
        #     pose_g_msg = Pose2D()
        #     pose_g_msg.x = self.current_plan[0][0]
        #     pose_g_msg.y = self.current_plan[0][1]
        #     if len(self.current_plan)>1:
        #         pose_g_msg.theta = np.arctan2(self.current_plan[1][1]-self.current_plan[0][1],self.current_plan[1][0]-self.current_plan[0][0])
        #     else:
        #         pose_g_msg.theta = self.theta_g
        #     self.nav_pose_pub.publish(pose_g_msg)
        #     return
        # else:
        #     # just stop
        #     cmd_x_dot = 0
        #     cmd_theta_dot = 0

        # # saving the last velocity for the controller
        # self.V_prev = cmd_x_dot
        # self.V_prev_t = rospy.get_rostime()

        # cmd_msg = Twist()
        # cmd_msg.linear.x = cmd_x_dot
        # cmd_msg.angular.z = cmd_theta_dot
        # self.nav_vel_pub.publish(cmd_msg)



if __name__ == '__main__':
    nav = Navigator()
    rospy.spin()
