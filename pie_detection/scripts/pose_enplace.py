#!/usr/bin/env python3

# Python modules
import cv2
import numpy as np
import tf
from tf2_ros.buffer import Buffer

# Ros modules
import rospy
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker

# Custom modules
from pie_detection.msg import CamPose
from cvthread import ProcessThread, cvThread, BufferQueue

# https://github.com/awesomebytes/occupancy_grid_python
class PoseEnplacer():
    def __init__(self) -> None:
        self.map_sub = rospy.Subscriber(
            '/map',
            OccupancyGrid,
            self.map_callback
        )
        self.object_pose_sub = rospy.Subscriber(
            '/detection/campose',
            CamPose,
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

    def map_callback(self, msg):
        self.map_queue.put(msg)

    def object_pose_callback(self, msg):
        depth = msg.mean_depth
        x_min = msg.x_min
        x_max = msg.x_max
        x = (x_max + x_min) /2

        trans, rot = self.robot_pose_on_map()

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = 'detected_objects'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = trans[0] + depth
        marker.pose.position.y = trans[1]
        marker.pose.position.z = trans[2]
        marker.pose.orientation.x = rot[0]
        marker.pose.orientation.y = rot[1]
        marker.pose.orientation.z = rot[2]
        marker.pose.orientation.w = rot[3]

        marker.scale.x = 1
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0 # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)

    def robot_pose_on_map(self):
        while True:
            try:
                (trans, rot) = self.listener.lookupTransform(
                    '/base_link',
                    '/map',
                    rospy.Time.now())
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print(e)
            else:
                break

        return trans, rot

def main():
    print("OpenCV version: %s" % cv2.__version__)
    rospy.init_node('pose_enplace_node')

    pose_enplacer = PoseEnplacer()
    tf_buffer = Buffer()
    listener = tf.TransformListener(tf_buffer)
    listener.waitForTransform('/base_link', '/map', rospy.Time.now(), rospy.Duration(5.0))
    pose_enplacer.listener = listener

    rospy.spin()

if __name__ == '__main__':
    main()