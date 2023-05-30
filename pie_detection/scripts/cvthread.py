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
    def __init__(
            self, queue_in_image, queue_in_data, queue_out_image, queue_out_data, window_name, image_process_func, publisher) -> None:
        threading.Thread.__init__(self)
        self.cvbridge = CvBridge()
        self.queue_in_image = queue_in_image
        self.queue_in_data = queue_in_data
        self.queue_out_image = queue_out_image
        self.queue_out_data = queue_out_data
        self.image = None
        self.window_name = window_name
        self.image_process_func = image_process_func
        self.publisher = publisher

    def run(self) -> None:
        # Create a single OpenCV window
        # cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        # TODO: get width and height from self.camera_info
        # cv2.resizeWindow(self.window_name, 800,600)

        while True:
            self.image = self.queue_in_image.get()
            if self.queue_in_data is None:
                _input = False
            else:
                _input = self.queue_in_data.get()
            
            img, _output = self.image_process_func(self.image, _input)

            self.queue_out_image.put(img)

            if (self.publisher is not None) and (_output is not None):
                self.publisher.publish(_output)
            else:
                self.queue_out_data.put(_output)
            
            # cv2_img = return_tuple[0]
            # try:
            #     img = self.cvbridge.cv2_to_compressed_imgmsg(cv2_img)
            # except CvBridgeError as e:
            #     print(e)
            # else:
            #     self.queue_out.put(_output)


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
                # Quit

                #TODO: def shutdonw function in main thread which stops daemon threads
                rospy.signal_shutdown('Quit')
