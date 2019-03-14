#!/usr/bin/env python
import rospy
import numpy as np
import tf
import tf.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker

# look up doc.
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


from std_msgs.msg import ColorRGBA
import matplotlib.pyplot as plt


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

def compute_ellipse_points(a, b):
    th = np.arange(0, 2*np.pi+1, 0.2)
    x = a * np.cos(th)
    y = b * np.sin(th)
    return np.stack([x,y])

class PuddleViz:
    def __init__(self):
        rospy.init_node("puddle_viz")
        self.puddle_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()
        self.puddle_mean = None
        self.puddle_viz_pub = rospy.Publisher("/viz/puddle", Marker, queue_size=10)
        self.puddle_world_pub = rospy.Publisher("puddle_world", Point[], queue_size=10)
        self.new_puddles = False
        self.puddle_thresh = 0.5
        self.puddle_means = []
        rospy.Subscriber("/velodyne_puddle_filter", PointCloud2, self.velodyne_callback)


    def velodyne_callback(self, msg):
        '''
        This is an example of how to process PointCloud2 data.
        pc2.read_points creates an _iterator_ object.
        '''
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
        #grab filtered points here to recognize multiple puddles

        x_filtered = x_coords[filtered_points]
        y_filtered = y_coords[filtered_points]
        z_filtered = z_coords[filtered_points]
        self.puddle_time = msg.header.stamp

        puddle_indicies_list = self.segment_points(self.xy_filtered)
        for indicies in puddle_indicies_list:
            if len(indicies) > MIN_POINTS:
                puddle_points = self.xy_filtered[indicies, :]
                self.puddle_means.append((np.mean(puddle_points[:,0]), np.mean(np.mean(puddle_points[:,1])), 0))
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
        if self.puddle_mean is not None:
            if self.new_puddles:
                pt = PointStamped()
                pt.header.frame_id = '/velodyne'
                pt.header.stamp = self.puddle_time
                pt.point.x = self.puddle_mean[0]
                pt.point.y = self.puddle_mean[1]
                pt.point.z = self.puddle_mean[2]

                try:
                    # send a tf transform of the puddle location in the map frame
                    self.tf_listener.waitForTransform("/map", '/velodyne', self.puddle_time, rospy.Duration(.05))
                    puddle_center_message = []
                    for i in range(len(self.puddle_means)):
                        puddle_mean = self.puddle_means[i]
                        chull = self.convex_hulls[i]
                        self.puddle_broadcaster.sendTransform((puddle_mean[0], puddle_mean[1], puddle_mean[2]), 
                                                               [0,0,0,1],
                                                               self.puddle_time,
                                                               "/puddle" + str(i),
                                                               "/map") 
                        puddle_center = Point(puddle_mean[0], puddle_mean[1], puddle_mean[2])
                        puddle_center_message.append(puddle_center)
                    self.puddle_world_pub.publish(puddle_center_message)
                    self.new_puddles = False

                except:
                    print("a looping error occured")
                    pass


    def run(self):
        rate = rospy.Rate(50) # 50 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


if __name__ == '__main__':
    puddle_viz = PuddleViz()
    puddle_viz.run()
