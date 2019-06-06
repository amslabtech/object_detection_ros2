import os
import cv2
import sys
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from object_detection.yolo import YOLO
from object_detection.utils import generate_colors, make_r_image
from PIL import Image as PIL_Image
from PIL import ImageDraw
from PIL import ImageFont
import colorsys
import matplotlib.pyplot as plt
import json

class ObjectDetection(Node):

    def __init__(self):
        super().__init__('traveller')
        self.i = 0
        self.bridge = CvBridge()
        self.pub_str = self.create_publisher(String, '/demo/objects')
        self.pub_img = self.create_publisher(Image, '/demo/r_image')
        self.sub = self.create_subscription(Image,'/demo/image_raw', self.locate)
        data_folder = "src/travel/object_detection/model_data/yolo3/coco/"
        
        yolo_args = {
            "model_path": data_folder+"yolo.h5",
            "score": 0.01,
            "anchors_path": data_folder+"anchors.txt",
            "classes_path": data_folder+"classes.txt",
                }

        self.yolo_args = dict((k, v) for k, v in yolo_args.items() if v)
        self.model = YOLO(**self.yolo_args)
        
        class_num = 0
        classes_path = os.path.expanduser(self.yolo_args["classes_path"])
        with open(classes_path) as f:
            class_num = len(f.readlines())
        colors = generate_colors(class_num)
        self.colors = colors

    def locate(self,oimg):
        try:
            img = self.bridge.imgmsg_to_cv2(oimg, "bgr8")
            hsv_img=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Object detection
            image = PIL_Image.fromarray(img)

            result = self.model.detect_image(image)

            objects = result['objects']

            r_image = make_r_image(image, objects, self.colors)
            
            result = np.asarray(r_image)

            self.pub_img.publish(self.bridge.cv2_to_imgmsg(result, "bgr8"))

        except CvBridgeError as e:
           print(e)

def main(args=None):

    rclpy.init(args=args)
    objectdetection = ObjectDetection()
    try:
        rclpy.spin(objectdetection)
    finally:
        if objectdetection not in locals():
            objectdetection.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

