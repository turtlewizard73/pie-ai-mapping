#!/usr/bin/env python3

# Python modules
import cv2
import numpy as np

# Ros modules
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import Twist, Point
import rospy

# Custom modules
from yolov3_net import YoloV3Net
from cvthread import ProcessThread, cvThread, BufferQueue

class ImageDetectorNode():
    def __init__(self) -> None:
        self.cvbridge = CvBridge()
        self.tf_net = YoloV3Net()
        self.queueSize = 1

        ## init rgb image utilites
        self.rgb_image_queue = BufferQueue(self.queueSize)
        self.detected_image_queue = BufferQueue(self.queueSize)  # img
        self.detected_data_queue = BufferQueue(self.queueSize)  # boxes[0], scores[0], classes[0], nums[0]
        self.rgb_image_sub = rospy.Subscriber(
            '/depth_camera/rgb/image_raw/compressed',
            CompressedImage,
            self.rgb_image_callback)
        self.rgb_camera_thread = ProcessThread(
            queue_in_image=self.rgb_image_queue,
            queue_in_data=None,
            queue_out_image=self.detected_image_queue,
            queue_out_data=self.detected_data_queue,
            window_name='rgb-image-detected',
            image_process_func=self.tf_net.detect,
        )
        self.rgb_camera_thread.setDaemon(True)
        self.rgb_camera_thread.start()

        ## init depth image utilites
        self.depth_image_queue = BufferQueue(self.queueSize)
        self.segmented_image_queue = BufferQueue(self.queueSize)
        self.segmented_data_queue = BufferQueue(self.queueSize)
        self.depth_image_sub = rospy.Subscriber(
            '/depth_camera/depth/image_raw',
            Image,
            self.depth_image_callback)
        self.depth_camera_thread = ProcessThread(
            queue_in_image=self.depth_image_queue,
            queue_in_data=self.detected_data_queue,
            queue_out_image=self.segmented_image_queue,
            queue_out_data=self.segmented_data_queue,
            window_name='depth-image-segmented',
            image_process_func=self.same,
        )
        self.depth_camera_thread.setDaemon(True)
        self.depth_camera_thread.start()

        self.display_thread = cvThread(
            window_names=['rgb-image-detected', 'depth-image-segmented'],
            queues=[self.detected_image_queue, self.segmented_image_queue]
        )
        self.display_thread.setDaemon(True)
        self.display_thread.start()
     
        self.detected_image_pub = rospy.Publisher(
            '/detection/image',
            CompressedImage,
            queue_size=1)
        
        self.segmented_image_pub = rospy.Publisher(
            '/detection/segmented',
            CompressedImage,
            queue_size=1)
        
    def rgb_image_callback(self, msg) -> None:
        try:
            # Convert your ROS Image message to OpenCV2
            cv2Img = self.cvbridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            self.rgb_image_queue.put(cv2Img)

    def depth_image_callback(self, msg) -> None:
        try:
            # Convert your ROS Image message to OpenCV2
            cv2Img = self.cvbridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            cv_image_array = np.array(cv2Img, dtype = np.dtype('f8'))
            cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)

        except CvBridgeError as e:
            print(e)
        else:
            self.depth_image_queue.put(cv_image_norm)

    def same(self, img, _input):
        image = img
        return [image, True]

    def segment(self, image, depth_image, boxes, scores, classes, nums):
        input_tuple = self.detected_image_queue.get()
        boxes, scores, classes, nums = input_tuple[1], input_tuple[2], input_tuple[3], input_tuple[4]
        boxes=np.array(boxes)
        f = 0.5
        depth_image = cv2.resize(depth_image, (0,0), fx = f, fy = f)

        for i in range(nums):
            mask_rectangle = np.zeros(image.shape[:2], np.float32)
            x1, y1 = tuple((boxes[i,0:2] * [image.shape[1], image.shape[0]]).astype(np.int32))
            x2, y2 = tuple((boxes[i,2:4] * [image.shape[1], image.shape[0]]).astype(np.int32))
            mask_rectangle[y1:y2, x1:x2] = 1.0
            segmented_depth_img = depth_image * mask_rectangle

            mean_distance = np.mean(segmented_depth_img)
            min_distance = np.nanmin(segmented_depth_img)
            max_distance = np.nanmax(segmented_depth_img)

        output = [segmented_depth_img, mean_distance, min_distance, max_distance]
        return output


def main():
    print("OpenCV version: %s" % cv2.__version__)

    image_detector_node = ImageDetectorNode()
    rospy.init_node('image_detector')
    rospy.spin()

if __name__ == '__main__':
    main()