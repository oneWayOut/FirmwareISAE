import serial
import threading
import time
import struct  # https://docs.python.org/3/library/struct.html



def write():
	x = -2000
	y = 0
	z = 300
	while True:
		time.sleep(1)  # sleep 1 seconds
		x+=1
		y+=1
		z+=1  # unit = cm
		sendBytes = b'\xa5\xa5'  ## data start flag
		sendBytes += struct.pack('hhh', x, y, z)
		com4.write(sendBytes)


def receive():
	while True:
		if com4.in_waiting:
			str1 = com4.read(com4.in_waiting).decode()
			if(str1 != ''):
				print(str1)



com4 = serial.Serial("COM16", 57600, timeout=2)  # change COM16 to serial dev name

print(com4.portstr)

th1 = threading.Thread(target=write)
th2 = threading.Thread(target=receive)

th1.start()
th2.start()

