# check the protocol documentation here:
# https://www.willow.co.uk/TCM2.6_Manual.pdf

import AriaPy
import serial
import time

TCM_PORT = '/dev/ttyUSB0'

# read directly through serial since AriaPy API is not able to get the values
sconn = serial.Serial(TCM_PORT, 9600)

# instantiate the compass direct
mag_obj = AriaPy.ArTCMCompassDirect(TCM_PORT)

# connect to the serial port
mag_obj.connect()
print('Connected')


# send command to get a contiunous stream of packets with magnetometer values
mag_obj.commandContinuousPackets()
# mag_obj.commandOnePacket() # one packet request

# wait for acquisition to start
time.sleep(2)

while (True):
    try:
        # get the magnetometer packet
        mag_data = sconn.read_all()
        print(mag_data)
        
        # values in microtesla
        X = Y = Z = 0
        if (mag_data):
            for index in range(len(mag_data)):
                if mag_data[index] == 'X':
                    for j in range(index, len(mag_data)):
                        if mag_data[j] == 'Y':
                            X = mag_data[index+1:j]
                            break

                if mag_data[index] == 'Y':
                    for j in range(index, len(mag_data)):
                        if mag_data[j] == 'Z':
                            Y = mag_data[index+1:j]
                            break

                if mag_data[index] == 'Z':
                    for j in range(index, len(mag_data)):
                        if mag_data[j] == '*' or mag_data[j] == 'E':
                            Z = mag_data[index+1:j]
                            break

            print(X)
            print(Y)
            print(Z)

        time.sleep(1)

    except:
        # send command to stop stream
        mag_obj.commandOff()
        # close serial connection
        sconn.close()
        raise
