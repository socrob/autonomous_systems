hokuyo rules
============

execute:

udevadm info -a -n /dev/ttyACM0 | grep KERNELS

and add the rule:

SUBSYSTEM=="tty", KERNEL=="ttyACM*", KERNELS=="1-1.2:1.0", SYMLINK+="mbot/hokuyo_rear", MODE="0666"

replace KERNELS with the proper value

This rule will assign a simlink based on the specific usb port in which the device is connected.

i.e. connect a hokuyo sensor in a specific usb port. Get the kernel of that port will make all hokuyos connected
there to have the rule. Therefore always connect the sensor to the same port!!
