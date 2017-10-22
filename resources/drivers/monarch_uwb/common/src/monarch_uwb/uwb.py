#!/usr/bin/env python

'''
 * Copyright [2017] <Instituto Superior Tecnico>
 *
 * Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)
 *
 * Open source driver for the Ultra Wide Band sensor by Eliko:
 *
 *     https://www.eliko.ee/products/kio-rtls/
 *
 * partially based on the one:
 * Created on 20.02.2014
 * by @author: Fialho
'''

import serial
import time
import numpy as np

class UWBDriver(object):
    def __init__(self, port):
        '''
        Ultra Wide Band low level serial port communication driver
        '''
        try:
            self.serial_port = serial.Serial(port, 230400, timeout=0)
        except serial.SerialException:
            self.log_err('Error while openning UWB tag serial port !')


    def log_err(self, error_msg):
        '''
        print to console in red color to indicate that an error has ocurred
        '''
        print('\x1b[6;31;40m' + error_msg + '\x1b[0m')


    def __write_to_serial_port__(self, data, action_msg='not specified'):
        '''
        internal function to write a command to the serial port
        it is not allowed for you to use this internal function (is private),
        instead use start_reading or stop_reading from this class
        if data was successfully written to the port you will receive a "2" from the write method
        input:
            data - the chars that you want to write to the serial port
            action_msg - for printing error msg in case of failure
        output:
            will write to the serial port
        '''
        # write the command to the serial port and compare its return value
        if self.serial_port.write(data + '\n') == 2:
            return True
        else:
            self.log_err('[Failed to write to serial port] Error msg: while trying to : ' + action_msg)
            return False


    def start_reading_acquisition(self):
        '''
        send "a" char to serial port to start the acquisition of readings
        '''
        return self.__write_to_serial_port__('a', 'start the acquisition of readings')


    def stop_reading_acquisition(self):
        '''
        send "b" char to serial port to stop the acquisition of readings
        '''
        return self.__write_to_serial_port__('b', 'stop the acquisition of readings')


    def get_information_from_device(self):
        '''
        send a "s" char to the serial port to get some information out of the device
        example response:

        ******************
        header_error........:0
        frame_sync_loss.....:0
        bad_CRC.............:0
        address_filter_error:1
        SFD_timeouts........:1
        preamble_timeouts...:0
        rx_frme_wait_timeout:1122
        TX frame sent.......:2244
        Half period warning.:0
        TX power-up warning.:0
        '''
        return self.__write_to_serial_port__('s', 'get information from the device')


    def set_antenna_delay(self, number):
        '''
        send a "0-9" char to set information from the device regarding the antenna delay
        example response:

        tx/rx antenna delays set to 16447

        NOTE: This command is recommended to experiment first in a serial terminal such as "cutecom"
        '''
        return self.__write_to_serial_port__(number, 'get antenna delay')


    def read_anchors(self, debug=False):
        '''
        Read the most recent distance from tag to anchors from serial port

        More in detail:
        1. Check if there is new data on the serial port
        2. Read all data stored in buffer
        3. split this big string by char '\n\r' : change of line
        4. trash away last reading that always comes empty
        5. separate anchor readings in its own list, one list for anchor A, one for B and one for C
        6. compute mean and variance
        return mean and variance of the three anchors
        '''
        if self.serial_port.inWaiting() == 0:
            # no new data has been received in the serial port
            return None
        # read all data in the serial port
        #data = self.serial_port.read_all()
        data = self.serial_port.read(self.serial_port.inWaiting())
        if debug:
            print '-- raw data --'
            print data
        # divide long received string into smaller substrings based on the \n\r char
        readings = data.split('\n\r')
        # trash away last empty reading
        readings = readings[:len(readings)-1]
        if debug:
            print '-- readings --'
            print readings
        # fetch the last n readins of anchor A, B and C
        groupA = []
        groupB = []
        groupC = []
        for anchor_string in readings:
            temp = anchor_string.split(' ')
            if len(temp) == 5:
                try:
                    groupA.append(float(temp[0])) # anchor A last n readings
                    groupB.append(float(temp[1])) # anchor A last n readings
                    groupC.append(float(temp[2])) # anchor A last n readings
                except:
                    pass
            else:
                if debug:
                    self.log_err('Error: Unknown command received from serial port')
        # compute mean of the group readings
        meanA = np.mean(groupA)
        meanB = np.mean(groupB)
        meanC = np.mean(groupC)
        if debug:
            print '-- mean, based on '+ str(len(groupA)) + ' readings --'
            print 'anchor A : ' + str(meanA)
            print 'anchor B : ' + str(meanB)
            print 'anchor C : ' + str(meanC)
        # compute variance of the group of readings
        varA = np.var(groupA)
        varB = np.var(groupB)
        varC = np.var(groupC)
        if debug:
            print '-- variance, based on '+ str(len(groupA)) + ' readings --'
            print 'anchor A : ' + str(varA)
            print 'anchor B : ' + str(varB)
            print 'anchor C : ' + str(varC)
        # return mean and variance
        return [meanA, varA], [meanB, varB], [meanC, varC]


    def close_port(self):
        '''
        use serial api to close the serial port
        '''
        self.serial_port.close()
