#!/usr/bin/env python
import rospy
import numpy as np
import tf
import tf.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PointStamped, Point, Polygon, PolygonStamped, Point32, Quaternion
from visualization_msgs.msg import Marker

# look up doc.
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


from std_msgs.msg import ColorRGBA
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull


'''
This code identifies shiny points on the floor (aka puddle) and estimates its position.
It looks straight ahead within a 60 degree view, and only RHO meters in front.
Also look at velodyne_filter.launch

It broadcasts a TF frame called puddle, and draws a circle about the mean of the puddle position.

Although this "works", it is not perfect and it can be significantly improved.
- Currently, it is very noisy, as it considers other points from far away places.
- Currently, the puddle size is fixed.
- What happens when there are multiple nearby puddles?
- Does not visualize the "viewing sector" of the robot.
'''

# min number of points to be considered a puddle 
MIN_POINTS = 3

# look ahead distance to search for puddles
RHO = 1.5

def compute_convex_hull(xy_filtered):
    hull = ConvexHull(xy_filtered)
    return hull


def initialize_puddle_marker():
    puddle_marker = Marker()
    puddle_marker.header.frame_id = "/map"
    puddle_marker.ns = "chull"
    puddle_marker.type = Marker.LINE_STRIP
    puddle_marker.scale.x = .1
    puddle_marker.frame_locked = True
    puddle_marker.color.g = 1
    puddle_marker.color.a = 1
    puddle_marker.pose.position = Point(0.0,0.0,0.0)
    puddle_marker.pose.orientation = Quaternion(0.0,0.0,0.0,1.0)
    return puddle_marker

class PuddleViz:
    def __init__(self):
        rospy.init_node("puddle_viz")
        self.puddle_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.puddle_mean = None
        self.puddle_viz_pub = rospy.Publisher("/viz/puddle", Marker, queue_size=10)
        self.puddle_marker = initialize_puddle_marker()
        self.xy_filtered = np.zeros((0,2))
        self.convex_hull = None
        self.new_puddle = False
        self.puddle_thresh = 0.5

        rospy.Subscriber("/sample_puddle", PointCloud2, self.velodyne_callback)


    def velodyne_callback(self, msg):
        '''
        This is an example of how to process PointCloud2 data.
        pc2.read_points creates an _iterator_ object.
        '''
        print('recieved point cloud data')
        lidar_info = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        num_points = len(list(lidar_info))
        lidar_info = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        x_coords = np.zeros(num_points)
        y_coords = np.zeros(num_points)
        z_coords = np.zeros(num_points)
        i = 0
        # looping through the point cloud
        for p in lidar_info:

            x_coords[i] = p[0]
            y_coords[i] = p[1]
            z_coords[i] = p[2]
            i += 1

        pt_ranges = np.hypot(x_coords, y_coords)
        pt_angles = np.arctan2(y_coords, x_coords)

        # filter based on range
        pts_within_range = (pt_ranges < RHO)

        # filter based on angle
        pts_within_angle = (pt_angles < np.pi/6) & (pt_angles > -np.pi/6)

        # filtered points
        filtered_points = pts_within_range & pts_within_angle
        # filtered_points = pts_within_range

        x_filtered = x_coords[filtered_points]
        y_filtered = y_coords[filtered_points]
        #trying to use self.x and y filtered for convex hull
        self.xy_filtered = np.column_stack((x_filtered,y_filtered))
        # print(self.xy_filtered)
        # self.y_filtered = y_filtered
        z_filtered = z_coords[filtered_points]
        self.puddle_time = msg.header.stamp

        if sum(filtered_points) > MIN_POINTS:
            puddle_list = self.segment_points(self.xy_filtered)
            # self.puddle_mean = (np.mean(x_filtered), np.mean(y_filtered), 0)
            # self.convex_hull = compute_convex_hull(self.xy_filtered)
            print('created puddle hulls')
            self.new_puddle = True

    def segment_points(self, filtered):
        num_points = filtered.shape[0]
        dist_to_all_others = np.ones((num_points, num_points))*np.inf # distance to self stored as infinity
        for i in range(num_points):
            for j in range(num_points):
                if i != j:
                    dist_to_all_others[i,j] = np.linalg.norm(filtered[i] - filtered[j])

        clusters = []
        queue = []
        unprocessed_points = range(num_points)
        while len(unprocessed_points)>0:
            i = unprocessed_points.pop()
            processed = []
            queue.append(i)
            for q in queue:
                processed.append(q)
                dists_to_q = dist_to_all_others[q,:]
                indicies_within_thresh = np.where(dists_to_q < self.puddle_thresh)[0].tolist()
                indicies_unprocessed = [x for x in indicies_within_thresh if x not in processed]
                queue.extend(indicies_unprocessed)
            clusters.append(np.array(queue))
            unprocessed_points = [j for j in unprocessed_points if j not in queue] # remove all points that were just added to the cluster
        return clusters

    def loop(self):
        if self.convex_hull is not None:
            if self.new_puddle:
                # send fixed transform to puddle center for simulated data
                self.puddle_broadcaster.sendTransform((self.puddle_mean[0], self.puddle_mean[1], 
                                                       self.puddle_mean[2]), 
                                                       [0,0,0,1],
                                                       self.puddle_time,
                                                       "/puddle",
                                                       "/map")
                print('transform sent')
                
                # make puddle marker
                zeros = np.zeros((self.xy_filtered[self.convex_hull.vertices].shape[0],))
                pointlist = self.xy_filtered[self.convex_hull.vertices]
                pointlist = np.vstack((pointlist, pointlist[0,:]))
                self.puddle_marker.points = []
                for p in pointlist:
                    self.puddle_marker.points.append(Point(p[0], p[1], 0))
                self.puddle_marker.points
                self.puddle_viz_pub.publish(self.puddle_marker)
                self.new_puddle = False

    def run(self):
        rate = rospy.Rate(25) # 25 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


if __name__ == '__main__':
    puddle_viz = PuddleViz()
    rospy.sleep(1)
    puddle_viz.run()