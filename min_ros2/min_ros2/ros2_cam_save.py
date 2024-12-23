import cv2
import os
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class Cam_saver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.sub_complaced_img = self.create_subscription(
            CompressedImage,
            'rgb_image/compressed_image',
            self.callback_rgb_img,
            10)
        self.save_path = '/home/min/move3_ws/src'
        self.folder = '1280_720_cam_img_2'
        self.cnt=100
    
    def callback_rgb_img(self,data):
        np_arr = np.frombuffer(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode to color image
        cv2.imshow('RGB Image', image_np)
        key = cv2.waitKey(1)  # Display the image until a keypress
        if key == ord('c'):
            img_path=os.path.join(self.save_path,self.folder,str(self.cnt)+'.jpg')
            if not os.path.exists(self.save_path+'/'+self.folder):
                os.makedirs(self.save_path+'/'+self.folder)
            cv2.imwrite(img_path,image_np)
            self.cnt+=1
            print(img_path)
            
        elif key == ord('q'):
            self.destroy_subscription()
            rclpy.shutdown()

def main():
    rclpy.init()
    node=Cam_saver()
    rclpy.spin(node)










