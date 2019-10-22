import serial
import numpy as np
import cv2
import tuoyuan
import threading
import time
from collections import Counter


event = threading.Event()
event1 = threading.Event()
flag = 1

def yy():
    global flag

def write():
    #success = 1
    c = 0
    timeF = 15
    count = 0
    SUMG = 0
    SUMR = 0
    SUMB = 0
    countG = 0
    countR = 0
    countB = 0
    outputstr = "No target"
    while True:

        success, img = cap.read()
        if (success):
            crop = img[0:1079, 400:1279] #test1
            #crop = img[0:1080, 900:1100] #test2
            if(event1.isSet() == True):
                event1.clear()
                print(countB)
                print(countR)
                print(countG)
                if (SUMR / countR < SUMB / countB and SUMR / countR < SUMG / countG):
                    outputstr = "The first target is red"
                if (SUMR / countR < SUMB / countB and SUMR / countR > SUMG / countG):
                    outputstr = "The second target is red"
                if (SUMR / countR > SUMB / countB and SUMR / countR < SUMG / countG):
                    outputstr = "The second target is red"
                if (SUMR / countR > SUMB / countB and SUMR / countR > SUMG / countG):
                    outputstr = "The third target is red"
                com6.write(outputstr.encode())
                count = 0
                SUMG = 0
                SUMR = 0
                SUMB = 0
                countG = 0
                countR = 0
                countB = 0

            if(c % timeF == 0):
                event.wait()
                tc = tuoyuan.getAimcolor(crop)
                if (tc == ("cyan") or tc ==("green")):
                    count = count + 1
                    countG = countG + 1
                    SUMG = SUMG + count
                if (tc == ("blue")):
                    count = count + 1
                    countB = countB +1
                    SUMB = SUMB + count
                if (tc == "red"):
                    count = count + 1
                    countR = countR +1
                    SUMR = SUMR + count
                com6.write(tc.encode())
            c = c + 1
        '''else:
            print(countB)
            print(countR)
            print(countG)
            if (SUMR/countR < SUMB/countB and SUMR/countR < SUMG/countG):
                outputstr = "The first target is red"
            if (SUMR/countR < SUMB/countB and SUMR/countR > SUMG/countG):
                outputstr = "The second target is red"
            if (SUMR/countR > SUMB/countB and SUMR/countR < SUMG/countG):
                outputstr = "The second target is red"
            if (SUMR/countR > SUMB/countB and SUMR/countR > SUMG/countG):
                outputstr = "The third target is red"
            com6.write(outputstr.encode())
            break
        '''
def receive():
    while True:
        if com6.in_waiting:
            str = com6.read(com6.in_waiting).decode()
            print(str)
            if(str == "1"):
                event.set()
            if(str == "2"):
                event.clear()
                event1.set()

cap=cv2.VideoCapture(0) #调视频，调摄像头则参数为0
#timeF =60 #隔50帧截取一次
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
#c = 0
#success = 1
timex = 0

com6 = serial.Serial("/dev/ttyUSB0",57600,timeout=timex)
print(com6.portstr)
th1 = threading.Thread(target=receive)
th2 = threading.Thread(target=write)

th1.start()
th2.start()


