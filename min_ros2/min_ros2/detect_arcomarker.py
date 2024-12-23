import cv2
import os
import time
import math
import rclpy
import numpy as np

from rclpy.node import Node
from numpy.linalg import inv
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Quaternion

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
        self.pub_move_pos = self.create_publisher(Quaternion,'/p64_point_theta',10)
        self.this_aruco_dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[desired_aruco_dictionary])
        self.this_aruco_parameters = cv2.aruco.DetectorParameters()
        self.k = np.array([[1.44613227e+03,   0.,        6.43600895e+02], 
                [  0.,         1.43724540e+03, 4.00241369e+02],
                [  0.,           0.,           1.        ]])
        self.d = np.array([[0.08091033, 0.39609752, -0.01333719, -0.01925617, -2.07791668]])
        self.info = {}
        ## 평균을 낼때 사용하는 counter
        self.robot_cnt = 0
        self.p62_cnt = 0
        self.p63_cnt = 0
        self.p64_cnt = 0

        ## 마커가 보이는지 확인하는 변수
        self.showing_robot_maker = False
        self.showing_62_maker = False
        self.showing_63_maker = False
        self.showing_64_maker = False
        self.showing_65_maker = False

        ## 중간값을 구하기 위한 변수
        self.med_point_robot=[[],[],[],[]]
        self.med_point_62=[[],[],[],[]]
        self.med_point_63=[[],[],[],[]]
        self.med_point_64=[[],[],[],[]]


        ## 시간 체크 할 변수
        self.start_show_robot = 0
        self.start_show_62 = 0
        self.start_show_63 = 0
        self.start_show_64 = 0
        self.start_show_65 = 0


    def change_point(self,from_point,change_point):
        # from_point 기준으로 하고자 하는 point
        # change_point 바꾸고자 하는 point
        R_T =from_point[0].T # 카메라 좌표를 65 좌표로 변경
        val_T2=np.array([[1.,0.,0.,-from_point[1][0]],
                [0.,1.,0.,-from_point[1][1]],
                [0.,0.,1.,-from_point[1][2]],
                [0.,0.,0.,1.]])
        
        val_P2=np.array([[change_point[1][0]],
                    [change_point[1][1]],
                    [change_point[1][2]],
                    [1]])
        point_t_tran2 = np.dot(val_T2,val_P2) #좌표의 이동에 대한 계산 부터 실행
        new_R = np.array([[R_T[0][0],R_T[0][1],R_T[0][2],0.],
                [R_T[1][0],R_T[1][1],R_T[1][2],0.],
                [R_T[2][0],R_T[2][1],R_T[2][2],0.],
                [0.,0.,0.,1.]])
        robot_point = np.dot(new_R,point_t_tran2)
        R = np.dot(R_T,change_point[0])

        return robot_point[0],robot_point[1],robot_point[2],R

    def avg_robot_point(self,R,point,limit_cnt):
        if self.showing_65_maker and self.showing_robot_maker:
            half_limit = limit_cnt//2
            theta,x,y,z=-1,-1,-1,-1
            
            if self.robot_cnt < limit_cnt:
                self.med_point_robot[0].append(R[0][0])
                self.med_point_robot[0].sort()
                self.med_point_robot[1].append(point[0])
                self.med_point_robot[1].sort()
                self.med_point_robot[2].append(point[1])
                self.med_point_robot[2].sort()
                self.med_point_robot[3].append(point[2])
                self.med_point_robot[3].sort()
                self.robot_cnt +=1
            elif self.robot_cnt == limit_cnt:
                
                val_cos = self.med_point_robot[0][half_limit]
                radian = math.acos(val_cos)
                theta = radian*(180/math.pi)

                x = self.med_point_robot[1][half_limit]
                y = self.med_point_robot[2][half_limit]
                z = self.med_point_robot[3][half_limit]
                print('[robot] x: {0}'.format(x))
                print('[robot] y: {0}'.format(y))
                print('[robot] z: {0}'.format(z))
                print('--------------------')
                self.med_point_robot = [[],[],[],[]]
                self.robot_cnt = 0
            return theta,x,y,z
                
        ## 만약 현재 값과 평균 값의 차이가 5cm 이상 차이가 난다면 다시 평균을 낸다

    def avg_62_point(self,R,point,limit_cnt):
        if self.showing_65_maker and self.showing_62_maker:
            half_limit = limit_cnt//2
            theta,x,y,z=-1,-1,-1,-1

            if self.p62_cnt < limit_cnt:
                self.med_point_62[0].append(R[0][0])
                self.med_point_62[0].sort()
                self.med_point_62[1].append(point[0])
                self.med_point_62[1].sort()
                self.med_point_62[2].append(point[1])
                self.med_point_62[2].sort()
                self.med_point_62[3].append(point[2])
                self.med_point_62[3].sort()
                self.p62_cnt +=1
            elif self.p62_cnt == limit_cnt:
                val_cos = self.med_point_63[0][half_limit]
                radian = math.acos(val_cos)
                theta = radian*(180/math.pi)

                x = self.med_point_62[1][half_limit]
                y = self.med_point_62[2][half_limit]
                z = self.med_point_62[3][half_limit]
                print('[62] x: {0}'.format(x))
                print('[62] y: {0}'.format(y))
                print('[62] z: {0}'.format(z))
                print('--------------------')
                self.med_point_62 = [[],[],[],[]]
                self.p62_cnt = 0
            return theta,x,y,z

    def avg_63_point(self,R,point,limit_cnt):
        if self.showing_65_maker and self.showing_63_maker:
            half_limit = limit_cnt//2
            theta,x,y,z=-1,-1,-1,-1

            if self.p63_cnt < limit_cnt:
                self.med_point_63[0].append(R[0][0])
                self.med_point_63[0].sort()
                self.med_point_63[1].append(point[0])
                self.med_point_63[1].sort()
                self.med_point_63[2].append(point[1])
                self.med_point_63[2].sort()
                self.med_point_63[3].append(point[2])
                self.med_point_63[3].sort()
                self.p63_cnt +=1
            elif self.p63_cnt == limit_cnt:

                val_cos = self.med_point_63[0][half_limit]
                radian = math.acos(val_cos)
                theta = radian*(180/math.pi)

                x = self.med_point_63[1][half_limit]
                y = self.med_point_63[2][half_limit]
                z = self.med_point_63[3][half_limit]
                print('[63] x: {0}'.format(x))
                print('[63] y: {0}'.format(y))
                print('[63] z: {0}'.format(z))
                print('--------------------')
                self.med_point_63 = [[],[],[],[]]
                self.p63_cnt = 0
            return theta,x,y,z

    def avg_64_point(self,R,point,limit_cnt):
        if self.showing_65_maker and self.showing_64_maker:
            half_limit = limit_cnt//2
            theta,x,y,z=-1,-1,-1,-1

            if self.p64_cnt < limit_cnt:
                self.med_point_64[0].append(R[0][2])
                self.med_point_64[0].sort()
                self.med_point_64[1].append(point[0])
                self.med_point_64[1].sort()
                self.med_point_64[2].append(point[1])
                self.med_point_64[2].sort()
                self.med_point_64[3].append(point[2])
                self.med_point_64[3].sort()
                self.p64_cnt +=1

            elif self.p64_cnt == limit_cnt:
                val_sin = self.med_point_64[0][half_limit]
                radian = math.asin(val_sin)
                theta = radian*(180/math.pi)

                x = self.med_point_64[1][half_limit]
                y = self.med_point_64[2][half_limit]
                z = self.med_point_64[3][half_limit]
                # print('[64] x: {0}'.format(x))
                # print('[64] y: {0}'.format(y))
                # print('[64] z: {0}'.format(z))
                # print('--------------------')
                self.med_point_64 = [[],[],[],[]]
                self.p64_cnt = 0
            return theta,x,y,z

    def callback_rgb_img(self,data):
        np_arr = np.frombuffer(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode to color image
        check_time = 3
        change_robot=[-1,-1,-1,-1]
        change_p62=[-1,-1,-1,-1]
        change_p63=[-1,-1,-1,-1]
        change_p64=[-1,-1,-1,-1]
        
        (corners, ids, rejected) = cv2.aruco.detectMarkers(
        image_np, self.this_aruco_dictionary, parameters=self.this_aruco_parameters)
        try:
            # if 61 in ids:
            #     self.showing_robot_maker = True
            #     self.start_show_robot = time.time()
            # else:
            #     end_time_robot = time.time()
            #     self.showing_robot_maker = False
            #     if end_time_robot - self.start_show_robot > check_time:
            #         print('robot 마커 시야에서 안보임')
            if 62 in ids:
                self.showing_62_maker = True
                self.start_show_62 = time.time()
            else:
                end_time_62 = time.time()
                self.showing_62_maker = False
                if end_time_62 - self.start_show_62 > check_time:
                    print('62번 마커 시야에서 안보임')
            # if 63 in ids:
            #     self.showing_63_maker = True
            #     self.start_show_63 = time.time()
            # else:
            #     end_time_63 = time.time()
            #     self.showing_63_maker = False
            #     if end_time_63 - self.start_show_63> check_time:
            #         print('63번 마커 시야에서 안보임')
            if 64 in ids:
                self.showing_64_maker = True
                self.start_show_64 = time.time()
            else:
                end_time_64 = time.time()
                self.showing_64_maker = False
                if end_time_64 - self.start_show_64 > check_time:
                    print('64번 마커 시야에서 안보임')
            if 65 in ids:
                self.showing_65_maker = True
                self.start_show_65 = time.time()
            else:
                end_time_65 = time.time()
                self.showing_65_maker = False
                if end_time_65 - self.start_show_65 > check_time:
                    print('기준점 마커 시야에서 안보임')
        except TypeError as e:
            pass
        
        # Check that at least one ArUco marker was detected
        if len(corners) > 0:
        # Flatten the ArUco IDs list
            ids = ids.flatten()
            rvecs1, tvecs1, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.105, self.k, self.d)
            for i in range(len(ids)):
                # 인식된 아르코 마커 id:[회전 행렬, 이동 행렬]로 표현
                R, _ = cv2.Rodrigues(rvecs1[i])
                self.info[ids[i]]=[R,tvecs1[i][0],rvecs1[i][0]]
                cv2.drawFrameAxes(image_np, self.k, self.d, rvecs1[i], tvecs1[i], 0.1)
            try:
                info_60 = self.info.get(60)
                info_61 = self.info.get(61)
                info_62 = self.info.get(62)
                info_63 = self.info.get(63)
                info_64 = self.info.get(64)
                info_65 = self.info.get(65)
                #cur_robot_point=self.change_point(info_65,info_61)#로봇의 위치
                cur_62_point=self.change_point(info_65,info_62)#마커 62번 위치
                # cur_63_point=self.change_point(info_65,info_63)#마커 63번 위치
                cur_64_point=self.change_point(info_65,info_64)#마커 64번 위치

                # print('x_robot={0},y_robot={1},z_robot={2}'.format(cur_robot_point[0],cur_robot_point[1],cur_robot_point[2]))
                # self.avg_point(cur_robot_point,self.robot_cnt,self.avg_point_robot) #동작 안함.
                try:
                    #change_robot = self.avg_robot_point(info_61[0],cur_robot_point,10)
                    # change_p62 = self.avg_62_point(info_62[0],cur_62_point,10)
                    # change_p63 = self.avg_63_point(info_63[0],cur_63_point,10)
                    change_p64 = self.avg_64_point(cur_64_point[3],cur_64_point,10)

                    if change_p64[0]!=-1 and change_p64[1]!=-1 and change_p64[2]!=-1 and change_p64[3]!=-1:
                        data = Quaternion()
                        data.x=float(change_p64[1])
                        data.y=float(change_p64[2])
                        data.z=float(change_p64[3])
                        data.w=float(-change_p64[0])
                        self.pub_move_pos.publish(data)
                except IndexError as d:
                    pass


            except TypeError as e:
                pass

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
3