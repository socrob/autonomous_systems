udev rules
===

udev allows you to detect USB plug in events on your PC to automatically take actions.

Such action could be for instance to give proper rights to the port and to create a simbolic link

on your computer.


how to figure out your own udev rule
===

do:

udevadm info -a -n /dev/ttyACM0
lsusb

and identify product ID, vendor ID or another attribute that could be usefull to distinguish between different USB devices.

The rule will follow a pattern like this one:

SUBSYSTEM=="tty",KERNEL=="ttyACM*",SYMLINK+="mbot/hokuyo_rear",MODE="0666"


make port specific udev rules
===

If you have 2 USB devices with the same vendor ID product ID and same serial

then finding an attribute that can uniquely identify them becomes hard.

The instructions below can help you to create udev rules which are specific to a particular USB port

on your system. i.e. the one next to your laptop fan. Being said that the device will always need to be connected

to that specific port.


instructions
===

execute:

udevadm info -a -n /dev/ttyACM0 | grep KERNELS

and add the rule:

SUBSYSTEM=="tty", KERNEL=="ttyACM*", KERNELS=="1-1.2:1.0", SYMLINK+="mbot/hokuyo_rear", MODE="0666"

replace KERNELS with the proper value

This rule will assign a simlink based on the specific usb port in which the device is connected.

i.e. connect your USB sensor in a specific usb port. Get the kernel of that port will make all sensors connected
there to have the rule. Therefore always connect the sensor to the same port!!
