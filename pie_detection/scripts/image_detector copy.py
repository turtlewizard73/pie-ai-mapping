#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import Twist, Point
import rospy
try:
    from queue import Queue
except ImportError:
    from Queue import Queue
import threading
import numpy as np

from yolov3_net import YoloV3Net

class BufferQueue(Queue):
    def put(self, item, *args, **kwargs) -> None:
        with self.mutex:
            if self.maxsize > 0 and self._qsize() == self.maxsize:
                self._get()
            self._put(item)
            self.unfinished_tasks += 1
            self.not_empty.notify()

class cvThread(threading.Thread):
    def __init__(self, cvbridge, queue, depth_queue, process_image, get_distance, image_pub) -> None:
        threading.Thread.__init__(self)
        self.cvbridge: CvBridge = cvbridge
        self.queue = queue
        self.depth_queue = depth_queue
        self.image = None
        self.process_image = process_image
        self.get_distance = get_distance
        self.image_pub = image_pub

        self.window_name = 'detected image'

    def run(self) -> None:
        # Create a single OpenCV window
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.namedWindow('segmented', cv2.WINDOW_NORMAL)
        cv2.namedWindow('depth', cv2.WINDOW_NORMAL)
        # cv2.resizeWindow(self.window_name, 800,600)

        while True:
            self.image = self.queue.get()
            self.depth_image = self.depth_queue.get()

            cv2_img, boxes, scores, classes, nums = self.process_image(self.image)
            img = self.cvbridge.cv2_to_compressed_imgmsg(cv2_img)
            self.image_pub.publish(img)

            segmented_img, mean_distance, min_distance, max_distance = self.get_distance(
                self.image,
                self.depth_image,
                boxes, scores, classes, nums
                )
            
            cv2.imshow(self.window_name, cv2_img)

            cv2.imshow('depth', self.depth_image)

            cv2.imshow('segmented', segmented_img)

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
    def __init__(self) -> None:
        self.cvbridge = CvBridge()
        self.tf_net = YoloV3Net()

        self.image_sub = rospy.Subscriber(
            '/depth_camera/rgb/image_raw/compressed',
            CompressedImage,
            self.image_callback)
        
        self.depth_image_sub = rospy.Subscriber(
            '/depth_camera/depth/image_raw',
            Image,
            self.depth_image_callback)
        
        self.camera_info = None
        self.camera_info_sub = rospy.Subscriber(
            '/depth_camera/depth/camera_info',
            CameraInfo,
            self.camera_info_callback)
        
        self.image_pub = rospy.Publisher(
            '/detection/image',
            CompressedImage,
            queue_size=1)
        
        self.pose_pub = rospy.Publisher(
            '/detection/pose',
            Point,
            queue_size=1)
        
        self.queueSize = 1
        self.qMono = BufferQueue(self.queueSize)
        self.qMono_depth = BufferQueue(self.queueSize)
        self.cvThreadHandle = cvThread(
            self.cvbridge,
            self.qMono,
            self.qMono_depth,
            self.tf_net.detect,
            self.get_distance,
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

    def depth_image_callback(self, msg) -> None:
        try:
            # Convert your ROS Image message to OpenCV2
            #cv2Img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8") # in case of non-compressed image stream only
            cv2Img = self.cvbridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            cv_image_array = np.array(cv2Img, dtype = np.dtype('f8'))
            cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)

        except CvBridgeError as e:
            print(e)
        else:
            self.qMono_depth.put(cv_image_norm)

    def camera_info_callback(self, msg):
        print(msg)
        self.camera_info = msg
        self.camera_info_sub.unregister()

    def get_distance(self, image, depth_image, boxes, scores, classes, nums):
        boxes, classes, nums = boxes[0], classes[0], nums[0]
        boxes=np.array(boxes)
        f = 0.5
        depth_image = cv2.resize(depth_image, (0,0), fx = f, fy = f)

        print(boxes)
        print(classes)
        print(nums)

        for i in range(nums):
            mask_rectangle = np.zeros(image.shape[:2], np.float32)
            x1, y1 = tuple((boxes[i,0:2] * [image.shape[1], image.shape[0]]).astype(np.int32))
            x2, y2 = tuple((boxes[i,2:4] * [image.shape[1], image.shape[0]]).astype(np.int32))
            mask_rectangle[y1:y2, x1:x2] = 1.0
            segmented_depth_img = depth_image * mask_rectangle

            mean_distance = np.mean(segmented_depth_img)
            min_distance = np.nanmin(segmented_depth_img)
            max_distance = np.nanmax(segmented_depth_img)

        return segmented_depth_img, mean_distance, min_distance, max_distance


def main():
    print("OpenCV version: %s" % cv2.__version__)

    image_detector_node = ImageDetectorNode()
    rospy.init_node('image_detector')
    rospy.spin()

if __name__ == '__main__':
    main()