import smbus
import time
import struct
import math
import numpy as np

###
# ALL GLOBAL VARIABLES USED IN FUNCTIONS MUST BE GIVEN THE global TAG
###

bus = smbus.SMBus(1)
accel_address = 0x53 # accelerometer I2C address
gyro_address = 0x68  # gyroscope I2C address
magn_address = 0x1e  # magnetometer I2C address

accel_xyz = np.empty([3])
gyro_xyz = np.empty([3])
magn_xyz = np.empty([3])

ACCEL_X_SCALE = 0.004
ACCEL_Y_SCALE = 0.004
ACCEL_Z_SCALE = 0.004
ACCEL_X_OFFSET = 0
ACCEL_Y_OFFSET = 0
ACCEL_Z_OFFSET = 0

GYRO_GAIN = 0.069565 # 1/14.375 : multiplication is faster than division
# GYRO_GAIN converts raw gyroscope data to deg/s values
# the gyroscope is calibrated every time the program starts, these
# variable are initialized here to make them global; the global tag is
# given in the functions where they are used
GYRO_AVERAGE_OFFSET_X = 0
GYRO_AVERAGE_OFFSET_Y = 0
GYRO_AVERAGE_OFFSET_Z = 0

# Manual magnetometer calibration
#MAGN_X_MAX = 430
#MAGN_X_MIN = -415
#MAGN_Y_MAX = 520
#MAGN_Y_MIN = -530
#MAGN_Z_MAX = 355
#MAGN_Z_MIN = -431
#MAGN_X_OFFSET = (MAGN_X_MIN + MAGN_X_MAX) / 2.
#MAGN_Y_OFFSET = (MAGN_Y_MIN + MAGN_Y_MAX) / 2.
#MAGN_Z_OFFSET = (MAGN_Z_MIN + MAGN_Z_MAX) / 2.
#MAGN_X_SCALE = 100. / (MAGN_X_MAX - MAGN_X_OFFSET)
#MAGN_Y_SCALE = 100. / (MAGN_Y_MAX - MAGN_Y_OFFSET)
#MAGN_Z_SCALE = 100. / (MAGN_Z_MAX - MAGN_Z_OFFSET)

def set_accel():
	
    bus.write_byte_data(accel_address, 0x2c, 0x0b) # BW_RATE 100Hz
    bus.write_byte_data(accel_address, 0x31, 0x00) # DATA_FORMAT +-2g 10-bit resolution 
    bus.write_byte_data(accel_address, 0x2d, 0x08) # Power Control Register measurement mode

def set_gyro():

    bus.write_byte_data(gyro_address, 0x15, 0x07) # SMPLRT_DIV - 125Hz (output sample rate)
    #bus.write_byte_data(gyro_address, 0x16, 0x1d) # DLPF_FS - +-2000deg/s ; # DLPF_CFG - low pass 10Hz, internal sample rate 1kHz
    bus.write_byte_data(gyro_address, 0x16, 0x1a) # DLPF_FS - +-2000deg/s ; # DLPF_CFG - low pass 98Hz, internal sample rate 1kHz

def set_magn():
    
    bus.write_byte_data(magn_address, 0x02, 0x00) # MODE continuous - 15Hz default
    bus.write_byte_data(magn_address, 0x00, 0x18) # Config_REG_A - Output rate 75Hz
    
def get_accel():
    
    accel = np.empty([3])
    accel_x = bytearray()
    accel_y = bytearray()
    accel_z = bytearray()

    accel_x.append(bus.read_byte_data(accel_address, 0x33))
    accel_x.append(bus.read_byte_data(accel_address, 0x32))
    accel[0] = struct.unpack('>h',bytes(accel_x))[0]

    accel_y.append(bus.read_byte_data(accel_address, 0x35))
    accel_y.append(bus.read_byte_data(accel_address, 0x34))
    accel[1] = struct.unpack('>h',bytes(accel_y))[0]

    accel_z.append(bus.read_byte_data(accel_address, 0x37))
    accel_z.append(bus.read_byte_data(accel_address, 0x36))
    accel[2] = struct.unpack('>h',bytes(accel_z))[0]

    return accel

def get_gyro():

    gyro = np.empty([3])
    gyro_x = bytearray()
    gyro_y = bytearray()
    gyro_z = bytearray()

    gyro_x.append(bus.read_byte_data(gyro_address, 0x1d)) # GYRO_XOUT_H
    gyro_x.append(bus.read_byte_data(gyro_address, 0x1e)) # GYRO_XOUT_L
    gyro[0] = struct.unpack('>h',bytes(gyro_x))[0]

    gyro_y.append(bus.read_byte_data(gyro_address, 0x1f)) # GYRO_YOUT_H
    gyro_y.append(bus.read_byte_data(gyro_address, 0x20)) # GYRO_YOUT_L
    gyro[1] = struct.unpack('>h',bytes(gyro_y))[0]

    gyro_z.append(bus.read_byte_data(gyro_address, 0x21)) # GYRO_ZOUT_H
    gyro_z.append(bus.read_byte_data(gyro_address, 0x22)) # GYRO_ZOUT_L
    gyro[2] = struct.unpack('>h',bytes(gyro_z))[0]

    return gyro
 
def get_magn():
    
    magn = np.empty([3])
    magn_x = bytearray()
    magn_y = bytearray()
    magn_z = bytearray()
    
    magn_x.append(bus.read_byte_data(magn_address, 0x03))
    magn_x.append(bus.read_byte_data(magn_address, 0x04))
    magn[0] = struct.unpack('>h',bytes(magn_x))[0]

    magn_y.append(bus.read_byte_data(magn_address, 0x05))
    magn_y.append(bus.read_byte_data(magn_address, 0x06))
    magn[1] = struct.unpack('>h',bytes(magn_y))[0]

    magn_z.append(bus.read_byte_data(magn_address, 0x07))
    magn_z.append(bus.read_byte_data(magn_address, 0x08))
    magn[2] = struct.unpack('>h',bytes(magn_z))[0]

    return magn

def compensate_sensor_errors():

    global accel_xyz
    global gyro_xyz
    global magn_xyz
    #global heading_rad
    #global heading_deg

    accel_xyz[0] = (accel_xyz[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE
    accel_xyz[1] = (accel_xyz[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE
    accel_xyz[2] = (accel_xyz[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE

    gyro_xyz[0] = (gyro_xyz[0] - GYRO_AVERAGE_OFFSET_X) * GYRO_GAIN
    gyro_xyz[1] = (gyro_xyz[1] - GYRO_AVERAGE_OFFSET_Y) * GYRO_GAIN
    gyro_xyz[2] = (gyro_xyz[2] - GYRO_AVERAGE_OFFSET_Z) * GYRO_GAIN

    #magn_xyz[0] = (magn_xyz[0] - MAGN_X_OFFSET) * MAGN_X_SCALE
    #magn_xyz[1] = (magn_xyz[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE
    #magn_xyz[2] = (magn_xyz[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE
    #heading_rad = math.atan2(magn_xyz[1], magn_xyz[0]) # compass heading (rad)
    #heading_deg = TO_DEG(heading_rad) # compass heading (deg)
    

def millis():
    return int(round(time.time()*1000))

def TO_RAD(x):
    return x * 0.01745329252 # *pi/180

def TO_DEG(x):
    return x * 57.2957795131 # *180/pi

def set_things_up():

    global GYRO_AVERAGE_OFFSET_X
    global GYRO_AVERAGE_OFFSET_Y
    global GYRO_AVERAGE_OFFSET_Z

    set_accel()
    set_gyro()
    set_magn()
    
    # gyro must be stationary for correct calibration
    gyro_offset = np.zeros(3)
    #for i in range(0,63):
    for i in range(0,255):
        gyro_result = get_gyro()
        gyro_offset[0] += gyro_result[0]
        gyro_offset[1] += gyro_result[1]
        gyro_offset[2] += gyro_result[2]

    #GYRO_AVERAGE_OFFSET_X = gyro_offset[0] * 0.015625 # 1/64
    #GYRO_AVERAGE_OFFSET_Y = gyro_offset[1] * 0.015625 # 1/64
    #GYRO_AVERAGE_OFFSET_Z = gyro_offset[2] * 0.015625 # 1/64
    GYRO_AVERAGE_OFFSET_X = gyro_offset[0] * 0.00390625 # 1/256
    GYRO_AVERAGE_OFFSET_Y = gyro_offset[1] * 0.00390625 # 1/256
    GYRO_AVERAGE_OFFSET_Z = gyro_offset[2] * 0.00390625 # 1/256

def read_sensors():
    global accel_xyz
    global gyro_xyz
    global magn_xyz
    accel_xyz = get_accel()
    gyro_xyz = get_gyro()
    magn_xyz = get_magn()

def print_sensors(accel_xyz,gyro_xyz,magn_xyz):
    print("accel = [%5.2f,%5.2f,%5.2f]G" % (accel_xyz[0] , accel_xyz[1] , accel_xyz[2]))
    print("gyro = [%6.1f,%6.1f,%6.1f]deg/s" % (gyro_xyz[0] , gyro_xyz[1] , gyro_xyz[2]))
    print("magn = [%6.1f,%6.1f,%6.1f]Gauss\n" % (magn_xyz[0], magn_xyz[2], magn_xyz[2]))

set_things_up()

timer = millis()

fh = open('sensor_data.dat', 'w') # open file in write mode

i = 0

print("Recording sensor data...")

#while True:
while (i < 200): # record 200 data points (100 seconds)
    #if millis() - timer >= 20: # Main loop runs at 50Hz
    if millis() - timer >= 500: # Main loop runs at 2Hz
        timer_old = timer
        timer = millis()
        
        read_sensors()
        compensate_sensor_errors()

        #print("heading = %3.0f degrees" % heading_deg)
        #print_sensors(accel_xyz,gyro_xyz,magn_xyz)
        # save values to text file for processing
        fh.write('%6.2f,%6.2f,%6.2f\n' % (accel_xyz[0] , accel_xyz[1] , accel_xyz[2]))
        fh.write('%6.1f,%6.1f,%6.1f\n' % (gyro_xyz[0] , gyro_xyz[1] , gyro_xyz[2]))
        fh.write('%6.1f,%6.1f,%6.1f\n\n' % (magn_xyz[0], magn_xyz[2], magn_xyz[2]))
        i = i + 1
        
fh.close() # close file

print("Done.")
