import cv2
import os
import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

desired_aruco_dictionary = "DICT_5X5_100"
 
# The different ArUco dictionaries built into the OpenCV library. 
ARUCO_DICT = {
  "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
  "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
  "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
  "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
  "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
  "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
  "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
  "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
  "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
  "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
  "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
  "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
  "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
  "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
  "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
  "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
  "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

class Cam_saver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.sub_complaced_img = self.create_subscription(
            CompressedImage,
            'rgb_image/compressed_image',
            self.callback_rgb_img,
            10)
        self.this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[desired_aruco_dictionary])
        self.this_aruco_parameters = cv2.aruco.DetectorParameters()
    
    def callback_rgb_img(self,data):
        np_arr = np.frombuffer(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode to color image
        (corners, ids, rejected) = cv2.aruco.detectMarkers(
        image_np, self.this_aruco_dictionary, parameters=self.this_aruco_parameters)
        
        # Check that at least one ArUco marker was detected
        if len(corners) > 0:
        # Flatten the ArUco IDs list
            ids = ids.flatten()
            
            # Loop over the detected ArUco corners
            for (marker_corner, marker_id) in zip(corners, ids):
            
                # Extract the marker corners
                corners = marker_corner.reshape((4, 2))
                (top_left, top_right, bottom_right, bottom_left) = corners
                
                # Convert the (x,y) coordinate pairs to integers
                top_right = (int(top_right[0]), int(top_right[1]))
                bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))
                
                # Draw the bounding box of the ArUco detection
                cv2.line(image_np, top_left, top_right, (0, 255, 0), 2)
                cv2.line(image_np, top_right, bottom_right, (0, 255, 0), 2)
                cv2.line(image_np, bottom_right, bottom_left, (0, 255, 0), 2)
                cv2.line(image_np, bottom_left, top_left, (0, 255, 0), 2)
                
                # Calculate and draw the center of the ArUco marker
                center_x = int((top_left[0] + bottom_right[0]) / 2.0)
                center_y = int((top_left[1] + bottom_right[1]) / 2.0)
                cv2.circle(image_np, (center_x, center_y), 4, (0, 0, 255), -1)
                
                # Draw the ArUco marker ID on the video frame
                # The ID is always located at the top_left of the ArUco marker
                cv2.putText(image_np, str(marker_id), 
                (top_left[0], top_left[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)
        cv2.imshow('RGB Image', image_np)
        key = cv2.waitKey(1)  # Display the image until a keypress
        if key == ord('q'):
            self.destroy_subscription()
            rclpy.shutdown()

def main():
    rclpy.init()
    node=Cam_saver()
    rclpy.spin(node)










