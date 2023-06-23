#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np
import torch

class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('luggage_detection')
        self.label_pub = rospy.Publisher('label', String, queue_size=10)
        self.z_milieu_pub = rospy.Publisher('z_milieu', Float32, queue_size=10)
        self.image_pub = rospy.Publisher('image', Image, queue_size=10)
        self.bridge = CvBridge()
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(self.config)
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

    def run(self):
        rate = rospy.Rate(10)  # 10Hz

        while not rospy.is_shutdown():
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            image = cv2.flip(color_image, 1)
            image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results_yolo = self.model(image)

            h, w, _ = image.shape

            for detection in results_yolo.xyxy[0]:
                x1_rect, y1_rect, x2_rect, y2_rect, confidence, class_id = detection
                label = self.model.names[int(class_id)]
                x_milieu = (x1_rect + x2_rect) / 2
                y_milieu = (y1_rect + y2_rect) / 2
                z_milieu = depth_frame.get_distance(int(x_milieu), int(y_milieu))

                if label in ["suitcase", "handbag", "backpack"]:
                    # Publication des données
                    self.label_pub.publish(label)
                    self.z_milieu_pub.publish(z_milieu)

            image_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            self.image_pub.publish(image_msg)

            rate.sleep()

        self.pipeline.stop()

if __name__ == '__main__':
    try:
        node = ObjectDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass