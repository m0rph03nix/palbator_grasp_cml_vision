#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32
from cv_bridge import CvBridge
import cv2
import pyrealsense2 as rs
import numpy as np
import torch
import tf2_ros
from geometry_msgs.msg import TransformStamped
from math import sin, pi

class ObjectDetectionNode:
    def __init__(self):
        rospy.init_node('luggage_detection')
        self.label_pub = rospy.Publisher('label', String, queue_size=10)
        self.z_milieu_pub = rospy.Publisher('z_milieu', Float32, queue_size=10)
        self.image_pub = rospy.Publisher('image', Image, queue_size=10)
        self.bridge = CvBridge()
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 10)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 10)
        self.pipeline.start(self.config)
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()


    def getRealXY(self, x_ref, y_ref, distance, img_w=640, img_h=480, HFovDeg=90, VFovDeg=65):
        
        HFov = HFovDeg * pi / 180.0  # Horizontal field of view of the RealSense D455
        VFov = VFovDeg * pi / 180.0
        #Phi = (HFov / 2.0) * ( (2*neck_x)/self.image_w + 1)  #Angle from the center of the camera to neck_x
        PhiX = (HFov / 2.0) *  (x_ref - img_w/2) / (img_w/2) #Angle from the center of the camera to neck_x
        PhiY = (VFov / 2.0) *  (y_ref - img_h/2) / (img_h/2)
        return (    distance * sin(PhiX)  ,     distance * sin(PhiY)   )


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

                x_real, y_real = self.getRealXY(x_milieu, y_milieu, z_milieu, w, h)

                if label in ["suitcase", "handbag", "backpack"]:
                    # Publication des données
                    self.label_pub.publish(label)
                    print("{0} at {1:.2f} m".format(label, z_milieu))    
                    self.z_milieu_pub.publish(z_milieu)

                    # Création de la transformation TF
                    transform = TransformStamped()
                    transform.header.stamp = rospy.Time.now()
                    transform.header.frame_id = 'col_camera_sensor_d455_link'  # Frame de référence global
                    transform.child_frame_id = "OBJ_" + label + '_link' # Frame de l'objet détecté 
                    transform.transform.translation.x = z_milieu
                    transform.transform.translation.y = x_real
                    transform.transform.translation.z = y_real
                    transform.transform.rotation.x = 0.0
                    transform.transform.rotation.y = 0.0
                    transform.transform.rotation.z = 0.0
                    transform.transform.rotation.w = 1.0
                    
                    # Diffusion de la transformation TF
                    self.tf_broadcaster.sendTransform(transform)            

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
