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
        # from image_publisher
        # self.sub_img = self.create_subscription(Image,'/amsl/demo/image_raw', self.image_sub)

        # from gazebo
        self.sub_img = self.create_subscription(Image,'/cam/custom_camera/image_raw', self.image_sub)

        self.sub_r_img = self.create_subscription(Image,'/amsl/demo/r_image', self.r_image_sub)
        self.sub_objects = self.create_subscription(String,'/amsl/demo/objects', self.objects_sub)

    def image_sub(self,oimg):
        print("image sub")
        try:
            img = self.bridge.imgmsg_to_cv2(oimg, "bgr8")
        except CvBridgeError as e:
           print(e)

        cv2.imshow("Image windowt",img)
        cv2.waitKey(3)

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
