#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist
import rospy
try:
    from queue import Queue
except ImportError:
    from Queue import Queue
import threading
import numpy as np

from yolov3_net import YoloV3Net

class BufferQueue(Queue):
    """Slight modification of the standard Queue that discards the oldest item
    when adding an item and the queue is full.
    """
    def put(self, item, *args, **kwargs) -> None:
        # The base implementation, for reference:
        # https://github.com/python/cpython/blob/2.7/Lib/Queue.py#L107
        # https://github.com/python/cpython/blob/3.8/Lib/queue.py#L121
        with self.mutex:
            if self.maxsize > 0 and self._qsize() == self.maxsize:
                self._get()
            self._put(item)
            self.unfinished_tasks += 1
            self.not_empty.notify()

class cvThread(threading.Thread):
    """
    Thread that displays and processes the current image
    It is its own thread so that all display can be done
    in one thread to overcome imshow limitations and
    https://github.com/ros-perception/image_pipeline/issues/85
    """
    def __init__(self, cvbridge, queue, process_func, image_pub) -> None:
        threading.Thread.__init__(self)
        self.cvbridge: CvBridge = cvbridge
        self.queue = queue
        self.image = None
        self.process_func = process_func
        self.image_pub = image_pub

        self.window_name = 'detected image'

    def run(self) -> None:
        # Create a single OpenCV window
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 800,600)

        while True:
            self.image = self.queue.get()

            # Process the current image
            # self.processImage(self.image)
            cv2Img = self.process_func(self.image)
            img = self.cvbridge.cv2_to_compressed_imgmsg(cv2Img)
            self.image_pub.publish(img)

            cv2.imshow(self.window_name, cv2Img)

            # Check for 'q' key to exit
            k = cv2.waitKey(6) & 0xFF
            if k in [27, ord('q')]:
                # Stop every motion
                # self.cmd_vel.linear.x = 0
                # self.cmd_vel.angular.z = 0
                # pub.publish(self.cmd_vel)
                # Quit
                rospy.signal_shutdown('Quit')

class ImageDetectorNode():
    def __init__(self, cvbridge) -> None:
        self.cvbridge = cvbridge
        self.tf_net = YoloV3Net()

        self.image_sub = rospy.Subscriber(
            '/rgb_camera/rgb/image_raw/compressed',
            CompressedImage,
            self.image_callback)
        
        self.image_pub = rospy.Publisher(
            '/detection/image',
            CompressedImage,
            queue_size=1)
        
        self.queueSize = 1
        self.qMono = BufferQueue(self.queueSize)
        self.cvThreadHandle = cvThread(
            self.cvbridge,
            self.qMono,
            self.tf_net.detect,
            self.image_pub)
        
        self.cvThreadHandle.setDaemon(True)
        self.cvThreadHandle.start()

    def image_callback(self, msg) -> None:
        try:
            # Convert your ROS Image message to OpenCV2
            #cv2Img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8") # in case of non-compressed image stream only
            cv2Img = self.cvbridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            self.qMono.put(cv2Img)

def main():
    print("OpenCV version: %s" % cv2.__version__)

    cvbridge = CvBridge()
    image_detector_node = ImageDetectorNode(cvbridge)
    rospy.init_node('image_detector')
    rospy.spin()

if __name__ == '__main__':
    main()