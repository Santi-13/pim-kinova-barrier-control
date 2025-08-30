import serial
from time import sleep

"""
Config UART section
"""
ser = serial.Serial("/dev/ttyAMA0", 115200)
print("Configuration ready")
print(ser.name)
newline = "\n"


# Obtaining values
value1 = 1.4172210953694113
value2 = 11.23

# Encoding values
value1n = str(value1)
value2n = str(value2)
print(value1n)
print(value2n)

i = 0
while True:
	try:
		ser.write(b'1 \n')
		ser.write((value1n.encode()+newline.encode()))
		ser.write(b'2 \n')
		ser.write((value2n.encode()+newline.encode()))
		sleep(0.03)
		i+=1
		if (i == 10):
			break
	except KeyboardInterrupt:
		break
print("closing")
ser.close()

