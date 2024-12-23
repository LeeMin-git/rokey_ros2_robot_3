import cv2
import os
import rclpy
import numpy as np
from rclpy.node import Node
from numpy.linalg import inv
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
        self.k = np.array([[1.44613227e+03,   0.,        6.43600895e+02], 
                [  0.,         1.43724540e+03, 4.00241369e+02],
                [  0.,           0.,           1.        ]])
        self.d = np.array([[0.08091033, 0.39609752, -0.01333719, -0.01925617, -2.07791668]])
        self.info = {}

    def change_point(self,from_point,cahnge_point):
        R_T =from_point[0].T # 카메라 좌표를 65 좌표로 변경
        val_T2=np.array([[1.,0.,0.,-from_point[1][0]],
                [0.,1.,0.,-from_point[1][1]],
                [0.,0.,1.,-from_point[1][2]],
                [0.,0.,0.,1.]])
        
        val_P2=np.array([[cahnge_point[1][0]],
                    [cahnge_point[1][1]],
                    [cahnge_point[1][2]],
                    [1]])
        point_t_tran2 = np.dot(val_T2,val_P2) #좌표의 이동에 대한 계산 부터 실행
        new_R = np.array([[R_T[0][0],R_T[0][1],R_T[0][2],0.],
                [R_T[1][0],R_T[1][1],R_T[1][2],0.],
                [R_T[2][0],R_T[2][1],R_T[2][2],0.],
                [0.,0.,0.,1.]])
        robot_point = np.dot(new_R,point_t_tran2)

        return robot_point


    def callback_rgb_img(self,data):
        np_arr = np.frombuffer(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode to color image
        gray = cv2.cvtColor(image_np,cv2.COLOR_BGR2GRAY)
        
        (corners, ids, rejected) = cv2.aruco.detectMarkers(
        image_np, self.this_aruco_dictionary, parameters=self.this_aruco_parameters)
        
        # Check that at least one ArUco marker was detected
        if len(corners) > 0:
        # Flatten the ArUco IDs list
            ids = ids.flatten()
            corners1, ids1, _ = cv2.aruco.detectMarkers(gray, self.this_aruco_dictionary, parameters=self.this_aruco_parameters)
            rvecs1, tvecs1, _ = cv2.aruco.estimatePoseSingleMarkers(corners1, 0.105, self.k, self.d)
            for i in range(len(ids)):
                # 인식된 아르코 마커 id:[회전 행렬, 이동 행렬]로 표현
                R, _ = cv2.Rodrigues(rvecs1[i])
                self.info[ids[i]]=[R,tvecs1[i][0],rvecs1[i][0]]
                cv2.drawFrameAxes(image_np, self.k, self.d, rvecs1[i], tvecs1[i], 0.1)
                # print(self.info) #debug
                # print('---------------')#debug

            for (marker_corner, marker_id) in zip(corners, ids):
                if len(self.info) >= 1:
                    try:
                        nfo_60 = self.info.get(60)
                        info_61 = self.info.get(61)
                        info_62 = self.info.get(62)
                        info_63 = self.info.get(63)
                        info_64 = self.info.get(64)
                        info_65 = self.info.get(65)
                        print(self.change_point(info_65,info_61))#debug 로봇 - 이동하고자 하는 마커의 좌표

                        # print(robot_point)#debug 로봇 - 이동하고자 하는 마커의 좌표
                        # print('ori:{0}'.format(point_t_tran2))
                        # print('test:{0}'.format(test))
                        # print('-----------------')#debug
                        # print('-----------------')
                        # print('62 tvec: {0}'.format(info_62[1]))
                        # print('62 rvec: {0}'.format(info_62[2]))
                        # print('-----------------')
                        # print('63 tvec: {0}'.format(info_63[1]))
                        # print('63 rvec: {0}'.format(info_63[2]))
                        # print('-----------------')
                        # print('64 tvec: {0}'.format(info_64[1]))
                        # print('64 rvec: {0}'.format(info_64[2]))
                    except TypeError:
                        pass
                    pass
            
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










