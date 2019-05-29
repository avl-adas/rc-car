#i = 0
#for i in range(50):
import serial
import time
ser = serial.Serial('/dev/ttyACM0', 115200)
while True:

	#used to be 115200
	#ser.write('{B100}{S20}')
	ser.write('{S-20}')
	#sleep 1 sec between loops
	time.sleep(1)
	ser.write('{S0}')
	time.sleep(1)
	ser.write('{S20}')
	time.sleep(1)



