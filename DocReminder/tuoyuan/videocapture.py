
#从摄像头获取图像并截取帧

import numpy as np
import cv2
import tuoyuan
cap=cv2.VideoCapture(0) #调用笔记本内置摄像头，所以参数为0，如果有其他的摄像头可以调整参数为1，2
#cap=cv2.VideoCapture(0)
t = cap.isOpened()
if t is True:
    print("视频读入成功")
else:
    print("视频读入失败")
timeF =50
c = 0
success = 1
while success:
    success,img = cap.read()
    print(img.shape)
    #crop = img[400:800, 800:1050]
    if(c % timeF ==0):
        tuoyuan.getAimcolor(img)
    c = c+1
while True:
    #从摄像头读取图片
    sucess,img=cap.read()

    #保持画面的持续。
    crop = img[400:800, 800:1050]
    #cv2.namedWindow("showwindow",0)
    #cv2.moveWindow("showwindow",500,500)
    cv2.imshow("showwindow", crop)
    
    k=cv2.waitKey(1)
    if k == 27:
        #通过esc键退出摄像
        cv2.destroyAllWindows()
        break
    else:
        timeF = 10
        while sucess:
            sucess, img = cap.read()
            if (c % timeF == 0):  # 每隔timeF帧进行存储操作
                cv2.imwrite('image/' + str(c) + '.jpg', img)
                # 存储为图像
            c = c + 1
        break

    if k == ord("s"):
        #crop = img[400:800, 800:1150]
        #通过s键保存图片，并退出。
        #cv2.imwrite('5'+'.jpg',crop)
        tuoyuan.getAimcolor(crop)
        #cv2.destroyAllWindows()'''


#关闭摄像头

#cap.release()
