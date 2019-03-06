#!/usr/bin/env python
import rospy
import math
import sys
import numpy as np


from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

def randomPointCloud():
    rospy.init_node('sample_puddle')
    pcl_pub = rospy.Publisher("/sample_puddle", PointCloud2, queue_size=10)
    rospy.loginfo("Initializing sample pcl2 publisher node...")
    #give time to roscore to make the connections
    rate = rospy.Rate(0.5) # 1hz
    while not rospy.is_shutdown():
        
        cloud_points = np.random.randn(500,2)*1.5
        cloud_points = np.hstack((cloud_points, np.zeros((500,1))))
        cloud_points = cloud_points.tolist()
        #header
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'map'
        #create pcl from points
        scaled_polygon_pcl = pcl2.create_cloud_xyz32(header, cloud_points)
        #publish    
        rospy.loginfo("happily publishing sample pointcloud.. !")
        pcl_pub.publish(scaled_polygon_pcl)
        rate.sleep()

if __name__ == '__main__':
    '''
    Sample code to publish a pcl2 with python
    '''
    try:
        randomPointCloud()

    except rospy.ROSInterruptException:
           pass