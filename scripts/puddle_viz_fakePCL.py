#!/usr/bin/env python
import rospy
import numpy as np
import tf
import tf.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PointStamped, Point, Polygon, PolygonStamped, Point32, Quaternion
from visualization_msgs.msg import Marker, MarkerArray

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
RHO = 10

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
        self.puddle_viz_pub = rospy.Publisher("/viz/puddle", MarkerArray, queue_size=10)
        self.xy_filtered = np.zeros((0,2))
        self.new_puddles = False
        self.puddle_thresh = 0.5
        self.puddle_means = []
        self.puddle_markers = []
        self.convex_hulls = []

        rospy.Subscriber("/sample_puddle", PointCloud2, self.velodyne_callback)


    def velodyne_callback(self, msg):
        '''
        This is an example of how to process PointCloud2 data.
        pc2.read_points creates an _iterator_ object.
        '''
        print('recieved point cloud data')
        self.puddle_means = []
        self.puddle_markers = []
        self.convex_hulls = []
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
        pts_within_angle = (pt_angles < np.pi) & (pt_angles > -np.pi)

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

        puddle_indicies_list = self.segment_points(self.xy_filtered)
        for indicies in puddle_indicies_list:
            if len(indicies) > MIN_POINTS:
                puddle_points = self.xy_filtered[indicies, :]
                self.puddle_means.append((np.mean(puddle_points[:,0]), np.mean(np.mean(puddle_points[:,1])), 0))
                self.convex_hulls.append(compute_convex_hull(puddle_points))
        print('created puddle hulls')
        self.new_puddles = True

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
            # print("unprocessed list: " + str(unprocessed_points))
            i = unprocessed_points.pop()
            # print("currently evaluating: " + str(i))
            queue.append(i)
            for q in queue:
                # print("queue: " + str(queue))
                # print("processing: " + str(q))
                dists_to_q = dist_to_all_others[q,:]
                indicies_within_thresh = np.where(dists_to_q < self.puddle_thresh)[0].tolist()
                # print("indicies_within_thresh: " + str(indicies_within_thresh))
                indicies_unqueued = [x for x in indicies_within_thresh if x not in queue]
                # print("indicies_unqueued: " + str(indicies_unqueued))
                queue.extend(indicies_unqueued)
            clusters.append(np.array(queue))
            unprocessed_points = [j for j in unprocessed_points if j not in queue] # remove all points that were just added to the cluster
            queue = []
        return clusters

    def loop(self):
        if self.convex_hulls is not None:
            if self.new_puddles:
                # send fixed transform to puddle center for simulated data
                print(self.xy_filtered)
                for i in range(len(self.puddle_means)):
                    puddle_mean = self.puddle_means[i]
                    chull = self.convex_hulls[i]
                    self.puddle_broadcaster.sendTransform((puddle_mean[0], puddle_mean[1], puddle_mean[2]), 
                                                           [0,0,0,1],
                                                           self.puddle_time,
                                                           "/puddle" + str(i),
                                                           "/map")
                    if i == 0:
                        pointlist = self.xy_filtered[chull.vertices, :]
                        pointlist = np.vstack((pointlist, pointlist[0,:]))
                        pointlist = pointlist
                        print(pointlist)
                        chull_marker = initialize_puddle_marker()
                        # chull_marker.header.frame_id = "/puddle" + str(i)
                        chull_marker.points = []
                        for p in pointlist:
                            chull_marker.points.append(Point(p[0], p[1], 0))
                        chull_marker.pose.position = Point(puddle_mean[0], puddle_mean[1], 0)
                        self.puddle_markers.append(chull_marker)
                self.puddle_viz_pub.publish(self.puddle_markers)
                self.new_puddles = False

    def run(self):
        rate = rospy.Rate(25) # 25 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


if __name__ == '__main__':
    puddle_viz = PuddleViz()
    rospy.sleep(1)
    puddle_viz.run()