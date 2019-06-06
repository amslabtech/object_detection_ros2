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
from PIL import Image as PIL_Image
from PIL import ImageDraw
from PIL import ImageFont

class Agent(Node):

    def __init__(self):
        super().__init__('traveller')
        self.bridge = CvBridge()
        self.sub_img = self.create_subscription(Image,'/demo/image_raw', self.image_sub)
        self.sub_r_img = self.create_subscription(Image,'/demo/r_image', self.r_image_sub)
        self.sub_objects = self.create_subscription(String,'/demo/objects', self.objects_sub)

    def image_sub(self,oimg):
        try:
            img = self.bridge.imgmsg_to_cv2(oimg, "bgr8")

        except CvBridgeError as e:
           print(e)

        cv2.imshow("Image windowt",img)
        cv2.waitKey(3)
    
    # def object_detection_sub(self,objects)

    def r_image_sub(self,r_img):
        try:
            img = self.bridge.imgmsg_to_cv2(r_img, "bgr8")
            cv2.namedWindow("result", cv2.WINDOW_NORMAL)
            cv2.imshow("result", img)

        except CvBridgeError as e:
           print(e)

    def objects_sub(self, otext):
        print(otext.data)

def main(args=None):

    rclpy.init(args=args)
    agent = Agent()
    try:
        rclpy.spin(agent)
    finally:
        if agent not in locals():
            agent.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
        cv2.waitKey(0)

if __name__ == '__main__':
    main()
