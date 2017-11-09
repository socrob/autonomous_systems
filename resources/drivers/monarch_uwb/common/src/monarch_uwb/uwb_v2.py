#!/usr/bin/env python

'''
Copyright [2017] <Instituto Superior Tecnico>

Author: Oscar Lima (olima@isr.tecnico.ulisboa.pt)

Open source driver for the Ultra Wide Band sensor by Eliko:

    https://www.eliko.ee/products/kio-rtls/

partially based on the one:
Created on 20.02.2014
by @author: Fialho
'''

import serial
import sys
import numpy as np

class UWBDriver(object):
    def __init__(self, port):
        '''
        Ultra Wide Band low level serial port communication driver
        '''
        try:
            self.serial_port = serial.Serial(port, 230400, timeout=0)
        except serial.SerialException:
            self.log_err('Error while openning UWB tag serial port ! please make sure that : ' + port + ' exists')
            sys.exit(0)


    def log_err(self, error_msg):
        '''
        print to console in red color to indicate that an error has ocurred
        '''
        print('\x1b[6;31;40m' + error_msg + '\x1b[0m')


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
        data = self.serial_port.read(self.serial_port.inWaiting())
        if debug:
            print '-- raw data --'
            print data
        # divide long received string into smaller substrings based on the \n\r char
        readings = data.split('\n\r')
        if debug:
            print '-- readings --'
            print readings
        # fetch the last n readins of anchor A, B and C
        groupA = []
        groupB = []
        groupC = []
        for reading in readings:
            temp = reading.split(' ')
            if len(temp) == 8:
                try:
                    groupA.append(float(temp[1])) # anchor A last n readings
                    groupB.append(float(temp[3])) # anchor A last n readings
                    groupC.append(float(temp[5])) # anchor A last n readings
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
