#!/usr/bin/env python

# testing the device in ipython commands

import serial

serial_port = serial.Serial('/dev/mbot/uwb', 230400, timeout=0)
