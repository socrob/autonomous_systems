# from AriaPy import *
import AriaPy
import sys

mag_obj = AriaPy.ArTCMCompassDirect('/dev/ttyUSB0')

if (mag_obj.connect()):
    print('Connected to magnetometer')
    a = mag_obj.getXMagnetic()
    b = mag_obj.getYMagnetic()
    c = mag_obj.getZMagnetic()
    print(a, b, c)
else:
    print('Not able to connect')