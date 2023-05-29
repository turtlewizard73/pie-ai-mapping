#!/usr/bin/env python3

# Python modules
import cv2
import numpy as np
import tf
import math
from tf2_ros.buffer import Buffer

# Ros modules
import rospy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

# Custom modules
from pie_detection.msg import CamPose, CamPoses
from cvthread import ProcessThread, cvThread, BufferQueue

# https://github.com/awesomebytes/occupancy_grid_python
class PoseEnplacer():
    def __init__(self) -> None:
        self.horizontal_fov = 1.047
        self.pixel_width = 640
        self.object_pose_sub = rospy.Subscriber(
            '/detection/camposes',
            CamPoses,
            self.object_pose_callback
        )

        self.listener = None

        self.queueSize = 1
        self.map_queue = BufferQueue(self.queueSize)
        self.object_pose_queue = BufferQueue(self.queueSize)
        self.robot_pose_queue = BufferQueue(self.queueSize)
        self.marker = Marker()

        self.marker_pub = rospy.Publisher(
            'visualization_marker',
            Marker,
            queue_size=1
        )

    def object_pose_callback(self, msg):
        print('cooking')
        i = 0
        for pose in msg.poses:
            print(pose)
            name = pose.name
            depth = pose.mean_depth * 5
            x_min = pose.x_min*2
            x_max = pose.x_max*2

            valid_width = 2*depth*math.atan(self.horizontal_fov/2)

            valid_x_min = x_min / self.pixel_width * valid_width
            valid_x_max = x_max / self.pixel_width * valid_width
            center = (valid_x_min + valid_x_max) / 2

            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = rospy.Time.now()
            marker.ns = name
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD

            marker.pose.position.x = depth
            marker.pose.position.y = -(-valid_width/2 + center)
            marker.pose.position.z = 0
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 0

            marker.scale.x = 0.2
            marker.scale.y = valid_x_max - valid_x_min
            marker.scale.z = 0.1
            marker.color.a = 1.0 # Don't forget to set the alpha!
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            i = i+1
            self.marker_pub.publish(marker)

    # def robot_pose_on_map(self):
    #     trans, rot = 0, 0
    #     try:
    #         (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
    #         print(e)
    #     return trans, rot
    

def main():
    print("OpenCV version: %s" % cv2.__version__)
    rospy.init_node('pose_enplace_node')

    pose_enplacer = PoseEnplacer()
    tf_buffer = Buffer()
    listener = tf.TransformListener(tf_buffer)


    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('turtle_tf_listener')

    # listener = tf.TransformListener()
    pose_enplacer = PoseEnplacer()
    rospy.spin()

    # rate = rospy.Rate(10.0)
    # while not rospy.is_shutdown():
    #     trans, rot = pose_enplacer.robot_pose_on_map()
    #     try:
    #         (trans,rot) = listener.lookupTransform('/base_link', '/map', rospy.Time(0))
    #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
    #         print(e)
    #         continue
    #     print(trans, rot)
    #     pose_enplacer.object_pose_callback(trans, rot)

    #     rate.sleep()