#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
from cv2 import aruco
import numpy as np
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Float64

class ArucoDetector:
    def __init__(self):
        rospy.init_node('detect_aruco', anonymous=True)
        self.bridge = CvBridge()
        self.image_subscriber = rospy.Subscriber('/camera/image/compressed', CompressedImage, self.image_callback)
        self.image_publisher = rospy.Publisher('/image_aruco', Image, queue_size=1)
        self.aruco_detected_distance = rospy.Publisher('/aruco_distance', Float64, queue_size=10)

        # Ajusta estos parámetros de calibración según tu cámara
        #self.camera_matrix = np.array([[101.85916, 0, 159.5],
        #                               [0, 101.85916, 119.5],
        #                               [0, 0, 1]])
        #self.dist_coeffs = np.array([0.0, 0.0, 0.0, 0.0]) 
        self.camera_matrix = np.array([[153.53337,   0.     , 150.50761],
                                      [0.       , 153.62598, 118.64754],
                                      [0.       ,   0.     ,   1.     ]])
        
        self.dist_coeffs = np.array([-0.318094, 0.090092, 0.000346, -0.000410, 0.0])
        rospy.loginfo("ArucoDetector Node Initialized")

    def image_callback(self, msg):
        rospy.loginfo("Image received")
        try:
            #np_arr = np.frombuffer(image_msg.data, np.uint8)
            #cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            dictionary = aruco.Dictionary_get(aruco.DICT_6X6_250)
            parameters = aruco.DetectorParameters_create()

            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray_image, dictionary, parameters=parameters)

            if ids is not None:
                for i in range(len(ids)):
                    rotation_vectors, translation_vectors, _ = aruco.estimatePoseSingleMarkers(corners[i], 0.05, self.camera_matrix, self.dist_coeffs)
                    aruco.drawDetectedMarkers(cv_image, corners, ids)
                    distancia = np.linalg.norm(translation_vectors[0])
                    rospy.loginfo(f"ArUco Marker {ids[i]} Detected at Distance: {distancia}m")
                    print(f"detected marker {ids[i]} at distance {distancia} meters")
                    #self.aruco_detected_publisher.publish("DETECTED")
                    self.aruco_detected_distance.publish(Float64(distancia))
            
            image_message = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.image_publisher.publish(image_message)
            rospy.loginfo("Image published")

        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))


if __name__ == '__main__':

    try:
        
        aruco_detector= ArucoDetector()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
