#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose2D
import tf

x = 0.0
y = 0.0
th = 0.0   

def loop():
    global x
    global y
    global th
        
    try:
        origin_frame = "/map"
        (translation,rotation) = trans_listener.lookupTransform(origin_frame, '/base_footprint', rospy.Time(0))
        x = translation[0]
        y = translation[1]
        euler = tf.transformations.euler_from_quaternion(rotation)
        th = euler[2]
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass
            
    message = Marker()
    message.header.frame_id = "/base_footprint"
    message.header.stamp = rospy.Time.now()
    message.ns = 'turtlebot_robot'
    message.id = 1
    message.type = Marker.CYLINDER #cylinder
    message.action = Marker.ADD #add/modify
    message.pose.position.x = 0.0
    message.pose.position.y = 0.0
    message.pose.position.z = 0.0
    message.pose.orientation.x = 0.0
    message.pose.orientation.y = 0.0
    message.pose.orientation.z = 0.0
    message.pose.orientation.w = 0.0
    message.scale.x = 0.15
    message.scale.y = 0.15
    message.scale.z = 0.5
    message.color.a = 1.0
    message.color.r = 64
    message.color.g = 89
    message.color.b = 255
        
    robot_pose.publish(message)
    
def nav_pose_callback_m(msg):
    goal = Marker()
    goal.header.frame_id = "/map"
    goal.header.stamp = rospy.Time.now()
    goal.ns = 'goal_marker'
    goal.id = 1
    goal.type = Marker.CUBE
    goal.action = Marker.ADD
    goal.pose.position.x = msg.x
    goal.pose.position.y = msg.y
    goal.pose.position.z = 0.0
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = msg.theta
    goal.scale.x = 0.15
    goal.scale.y = 0.15
    goal.scale.z = 0.15
    goal.color.a = 1.0
    goal.color.r = 1.0
    goal.color.g = 0.0
    goal.color.b = 0.0
    
    pose_goal_marker.publish(goal)
    
if __name__ == '__main__':
    rospy.init_node('robot_marker', anonymous=True)
    trans_listener = tf.TransformListener()
    robot_pose = rospy.Publisher('robot_pose_marker', Marker, queue_size=10)
    pose_goal_marker = rospy.Publisher('pose_goal_marker', Marker, queue_size=10) 
    rospy.Subscriber('/nav_pose', Pose2D, nav_pose_callback_m)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        loop()
        rate.sleep()
