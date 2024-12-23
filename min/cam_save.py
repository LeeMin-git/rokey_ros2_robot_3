import cv2 as cv
import os
cap = cv.VideoCapture(4)
cnt = 0
while cap.isOpened():
    _,img = cap.read()
    h,w=img.shape[:2]
    cv.imshow('img',img)
    key = cv.waitKey(1)
    folder = 'maker_cam_img'
    if key == ord('c'):
        img_path=os.path.join(folder,str(cnt)+'.jpg')
        if not os.path.exists(folder):
            os.makedirs(folder)
        cv.imwrite(img_path,img)
        print(img_path)
        cnt+=1
    elif key == ord('q'):
        cap.release()
        cv.destroyAllWindows()