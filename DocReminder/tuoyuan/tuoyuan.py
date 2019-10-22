import cv2
import numpy as np
from matplotlib import pyplot as plt
import math
import time
import ColorD

def dilate_demo(image):  #膨胀
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    dst = cv2.dilate(image, kernel)
    return dst


def erode_demo(image):#腐蚀
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
    dst = cv2.erode(image, kernel)
    return dst





def getAimcolor(img):

    start = time.clock()
    targetcolor = "NO target"

    #crop = img[0:800, 800:1100]
    #cv2.namedWindow("showwindow",0)
    #cv2.moveWindow("showwindow",500,500)

    #img=cv2.imread("2.jpg", 3)
    #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    #cv2.imshow("huidu",gray)
    #ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    #cv2.imshow("1212",img)
    #img=cv2.blur(img, (1, 1))
    imgray=cv2.Canny(img,200,300,3) #检测边缘，并返回一个二值图
    '''cv2.namedWindow("1", 0)
    cv2.resizeWindow("1",700,700)
    cv2.imshow("1", imgray)'''

    for i in range(0,1):
        imgray = dilate_demo(imgray)
        imgray = erode_demo(imgray)

    cv2.namedWindow("2", 0)
    cv2.resizeWindow("2", 700, 700)
    cv2.imshow("2", imgray)
    #cv2.imshow("pengzhang", imgray2)

    ret,thresh = cv2.threshold(imgray,127,255,cv2.THRESH_BINARY)
    #cv2.imshow("kk",thresh);

    image, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #输入分别为二值图，轮廓检索模式（等级树结构轮廓），轮廓的近似方法（压缩水平方向，垂直方向，对角线方向的元素，只保留该方向的终点坐标，例如一个矩形轮廓只需4个点来保存轮廓信息）
    #输出分别为image为轮廓图；contours为一个list其中包含了每个轮廓中的坐标,len(contours)则表示轮廓的个数；hierarchy表示分别表示后一个轮廓、前一个轮廓、父轮廓、内嵌轮廓的索引编号，如果没有对应项，则该值为负数
    #cv2.imshow("2", image)

    #print(len(contours))

    list1 = []#储存中心点坐标
    #print(len(list1))

    count1=0
    count2=0
    for cnt in contours:
        if len(cnt)>100:
            # print(len(cnt))
            S1=cv2.contourArea(cnt)  #轮廓面积
            ell=cv2.fitEllipse(cnt)  #拟合出一个椭圆尽量使得点都在圆上，返回值为（中心坐标，（短轴，长轴），旋转角度）
            #print(ell)
            S2 =math.pi*ell[1][0]*ell[1][1]*1/4#求拟合椭圆的面积

            if (S1/S2)>0.9 and (S1/S2)<1.9:
                #print(ell)
                img = cv2.ellipse(img,ell,(0, 255, 0), 2)#在原图标出拟合椭圆位置
                #print(str(S1) + "    " + str(S2)+"   "+str(ell[0][0])+"   "+str(ell[0][1]))

                if len(list1) == 0:
                    list1.append((int(ell[0][0]),int(ell[0][1])))
                    crop = img[list1[len(list1) - 1][1] - 2:list1[len(list1) - 1][1] + 2,list1[len(list1) - 1][0] - 2:list1[len(list1) - 1][0] + 2]
                    #print(ColorD.get_color(crop))
                    targetcolor = ColorD.get_color(crop)
                    '''if targetcolor == 'red':
                        count1 = count1 + 1
                        count2 = count1
                    if targetcolor == 'blue' or targetcolor == 'green':
                        count1 = count1 + 1'''

                if len(list1) > 0:
                    if abs(list1[len(list1)-1][0]-int(ell[0][0])) >25 or abs(list1[len(list1)-1][1]-int(ell[0][1])) > 25:
                        list1.append((int(ell[0][0]), int(ell[0][1])))
                        crop = img[list1[len(list1)-1][1] - 2:list1[len(list1)-1][1] + 2, list1[len(list1)-1][0] - 2:list1[len(list1)-1][0] + 2]
                        #print(ColorD.get_color(crop))
                        targetcolor = ColorD.get_color(crop)
                        '''if ColorD.get_color(crop)=='red':
                            count1 = count1+1
                            count2 = count1
                        if targetcolor == 'blue' or targetcolor == 'green':
                            count1 = count1+1'''#1018

    #print(list1)
    #print(len(list1))
    #print(count2)
    end = time.clock()
    cv2.namedWindow("0", 0)
    cv2.resizeWindow("0", 700, 700)
    cv2.imshow("0",img)
    cv2.waitKey(20)
    return targetcolor


#img=cv2.imread("1.jpg", 3)
#crop = img[800:2000, 1000:1500]
#getAimcolor(img)
