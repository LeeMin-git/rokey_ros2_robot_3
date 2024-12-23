import matplotlib.pyplot as plt
import numpy as np
import cv2
from numpy.linalg import inv
import math
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
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}
Type = 'DICT_5X5_100'
k = np.array([[1.44613227e+03,   0.,        6.43600895e+02], 
                [  0.,         1.43724540e+03, 4.00241369e+02],
                [  0.,           0.,           1.        ]])
d = np.array([[0.08091033, 0.39609752, -0.01333719, -0.01925617, -2.07791668]])
def get_matrix(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[Type])
    arucoParams = cv2.aruco.DetectorParameters()

    corners1, ids1, _ = cv2.aruco.detectMarkers(gray, arucoDict, parameters=arucoParams)
    rvecs1, tvecs1, _ = cv2.aruco.estimatePoseSingleMarkers(corners1, 0.105, k, d)
    R, _ = cv2.Rodrigues(rvecs1[0])

    return ids1, rvecs1, R, tvecs1
unit_x = np.array([[1.0], [0.0], [0.0]])
unit_y = np.array([[0.0], [1.0], [0.0]])
unit_z = np.array([[0.0], [0.0], [1.0]])
frame = cv2.imread('./1280_720_maker_img/100.jpg')

ids1, rvecs1, R, tvecs1 = get_matrix(frame)
print('R: {0}'.format(R))
print('tvecs1: {0}'.format(tvecs1))
print('------------------')
print(ids1, rvecs1, R, tvecs1)

cv2.drawFrameAxes(frame, k, d, rvecs1[0], tvecs1[0], 0.1)

x_T = np.dot(R, unit_x)
y_T = np.dot(R, unit_y)
z_T = np.dot(R, unit_z)

axis_size = 100
axis_center = (150, 150)
cv2.line(frame, axis_center, (int(axis_center[0] + axis_size*x_T[0]), int(axis_center[1] + axis_size*x_T[1])), (0, 0, 255), 2)
cv2.line(frame, axis_center, (int(axis_center[0] + axis_size*y_T[0]), int(axis_center[1] + axis_size*y_T[1])), (0, 255, 0), 2)
cv2.line(frame, axis_center, (int(axis_center[0] + axis_size*z_T[0]), int(axis_center[1] + axis_size*z_T[1])), (255, 0, 0), 2)
cv2.putText(frame,str(round(tvecs1[0][0][2],4)),(int(axis_center[0] + axis_size*x_T[0]), int(axis_center[1] + axis_size*x_T[1])-10),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 0, 255), 2)
cv2.putText(frame,str(round(tvecs1[0][0][1],4)),(int(axis_center[0] + axis_size*y_T[0]), int(axis_center[1] + axis_size*y_T[1])-10),cv2.FONT_HERSHEY_SIMPLEX,1,(0, 255, 0), 2)
cv2.putText(frame,str(round(tvecs1[0][0][0],4)),(int(axis_center[0] + axis_size*z_T[0]), int(axis_center[1] + axis_size*z_T[1])-10),cv2.FONT_HERSHEY_SIMPLEX,1,(255, 0, 0), 2)

# print(x_T.T, y_T.T, z_T.T)
# print('R_T= {0}'.format(R.T))
# print(tvecs1[0][0].T)
print('tvecs1: {0}'.format(tvecs1[0][0]))
data1 = np.dot(R.T,tvecs1[0][0].T)
print('data1:{0}'.format(data1))
data2 = np.dot(R,data1)
print('data2:{0}'.format(data2))

plt.figure(figsize=(10,10))
plt.imshow(frame)
plt.show()

