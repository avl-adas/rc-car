import serial
ser = serial.Serial('/dev/ttyACM0', 9600)
#used to be 115200
#ser.write('{S20}')
#ser.write('{S00}')
