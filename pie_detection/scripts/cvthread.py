#!/usr/bin/env python3

import cv2
import threading
import rospy
from cv_bridge import CvBridge, CvBridgeError
try:
    from queue import Queue
except ImportError:
    from Queue import Queue

import multiprocessing

class BufferQueue(Queue):
    def put(self, item, *args, **kwargs) -> None:
        with self.mutex:
            if self.maxsize > 0 and self._qsize() == self.maxsize:
                self._get()
            self._put(item)
            self.unfinished_tasks += 1
            self.not_empty.notify()

class ProcessThread(threading.Thread):
    def __init__(self, queue_in, queue_out, window_name, image_process_func) -> None:
        threading.Thread.__init__(self)
        self.cvbridge = CvBridge()
        self.queue_in = queue_in
        self.queue_out = queue_out
        self.image = None
        self.window_name = window_name
        self.image_process_func = image_process_func

    def run(self) -> None:
        # Create a single OpenCV window
        # cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        # TODO: get width and height from self.camera_info
        # cv2.resizeWindow(self.window_name, 800,600)

        while True:
            self.image = self.queue_in.get()
            cv2_img = self.image_process_func(self.image)
            self.queue_out.put(cv2_img)

            # cv2_img = return_tuple[0]
            # try:
            #     img = self.cvbridge.cv2_to_compressed_imgmsg(cv2_img)
            # except CvBridgeError as e:
            #     print(e)
            # else:
            #     self.queue_out.put(img)


class cvThread(threading.Thread):
    def __init__(self, window_names: list, queues: list):
        threading.Thread.__init__(self)
        self.window_names = window_names
        self.queues = queues
        self.images = []

    def run(self):
        # Create a single OpenCV window
        for name in self.window_names:
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)
            # cv2.resizeWindow("frame", 800,600)

        while True:
            self.images = []
            for queue, name in zip(self.queues, self.window_names):
                image = queue.get()
                cv2.imshow(name, image)

            # Process the current image
            # self.processImage(self.image)

            # cv2.imshow("frame", self.image)

            # Check for 'q' key to exit
            k = cv2.waitKey(6) & 0xFF
            if k in [27, ord('q')]:
                # Stop every motion
                self.cmd_vel.linear.x = 0
                self.cmd_vel.angular.z = 0
                pub.publish(self.cmd_vel)
                # Quit
                rospy.signal_shutdown('Quit')