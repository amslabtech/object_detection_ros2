import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pynput.keyboard import Key, Listener
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import time
from PIL import Image as PIL_Image
from PIL import ImageDraw
from PIL import ImageFont
import numpy as np
import cv2
 
class Img(Node):

    def __init__(self):
        super().__init__('text')
        self.pub = self.create_publisher(Image, '/amsl/demo/image_raw')
        self.bridge = CvBridge()
        self.check()


    def check(self):
        vid = cv2.VideoCapture(0)
        if not vid.isOpened():
            raise IOError("Couldn't open webcam or video")

        while True:
            print("vid.read()")
            return_value, frame = vid.read()
            image = PIL_Image.fromarray(frame)
            image = np.asarray(image)
            self.pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))
            print("pub done")


def main(args=None):

    rclpy.init(args=args)
    img = Img()
    try:
        rclpy.spin(img)
    finally:
        if img not in locals():
            img.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
