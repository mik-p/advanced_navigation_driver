# advanced_navigation_driver
Driver for the range of INS systems from Advanced Navigation

Packet to Published Message Example

Copyright 2017, Advanced Navigation

This is an example using the Advanced Navigation Spatial SDK to create a ROS driver that reads and decodes the anpp packets (in this case packet #20 and packet #27) and publishes the information as ROS topics / messages. 

It should work on all Advanced Navigation INS devices.

This example has been developed and tested using Ubuntu Linux v16.04 LTS and ROS Lunar. Installation instructions for ROS can be found here: http://wiki.ros.org/lunar/Installation/Ubuntu.

If you require any assistance using this code, please email support@advancednavigation.com.au.

Installation, build, device configuration, and execution instructions can be found in the file "Advanced Navigation ROS Driver Notes.txt".

# Baud rate

## Change the baud rate

The driver now includes a script that allows for changing the baud rate (`src/an_change_baudrate.c`). It requires the com port, the old baudrate and the new baudrate as arguments. It doesn't perform any checks if those are correct or if the change in baudrate actually worked. For checking make sure that the `advanced_navigation_driver` actually publishes valid topics in ros with the new baudrate.

## Remark on baud rate and packet rate

According to the manual, when configuring packet rates it is essential to ensure the baud rate is capable of handling the data throughput. This can be calculated using the rate and packet size. The packet size is the packet length add five to account for the packet overhead. For example to output the system state packet at 50Hz the calculation would be:

* Data throughput = (100 (packet length) + 5 (fixed packet overhead)) * 50 (rate)
* Data throughput = 5250 bytes per second
* Minimum baud rate = data throughput x 11 = 57750 Baud
* Closest standard baud rate = 115200 Baud

When multiple packets are set to output at the same rate, the order the packets output is from lowest ID to highest ID. It seems (just from experience) that if the rate is set to high for a given baud rate then only as many packets are send as the current baudrate supports, the remaining packets will be missing.

# Settings

* At this point the driver requires packets 20, 27 and 28. These packets are reconfigured automatically by the driver. Any previous configuration is lost.
