import serial
from math import pi
from time import sleep



def rpm(ms, w):
    v_rpm = ms/w*2*pi/60
    return round(v_rpm, 6)

"""
Config section
"""
ser = serial.Serial("/dev/ttyAMA0", 115200)
print("Configuration ready")
print(ser.name)
newline = "\n"

"""
Robot Kinematics Definition
"""

V = 1.1
w = 1.1

l = 0.5
w_r = 0.1016

vr = (2*V + l*w)/2
vl = (2*V - l*w)/2

vr_rpm = rpm(vr, w_r)
vl_rpm = rpm(vl, w_r)


"""
UART Encoding and communication
"""
# Encoding values
vrn = str(vr_rpm)
vln = str(vl_rpm)
print(vrn)
print(vln)

i = 0

while True:
    try:
        ser.write(b'1 \n')
        ser.write((vrn.encode()+newline.encode()))
        ser.write(b'2 \n')
        ser.write((vln.encode()+newline.encode()))
        sleep(0.01)

    except KeyboardInterrupt:
        break

print("closing")
ser.close()