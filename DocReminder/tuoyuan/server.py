import serial
import threading
import time

def write():
    while True:
        str = input()
        com4.write(str.encode())
def receive():
    while True:
        if com4.in_waiting:
            while True:
                str1 = com4.read(com4.in_waiting).decode()
                if(str1 != ''):
                    print(str1)


timex = 0
com4 = serial.Serial("/dev/ttyUSB1", 57600, timeout=timex)

print(com4.portstr)

th1 = threading.Thread(target=write)
th2 = threading.Thread(target=receive)
th1.start()
th2.start()
