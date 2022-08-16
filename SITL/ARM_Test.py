import StartLidar_New
import SuperDrone 
import time

#for the dronekit imports necessary to run the utilities of SuperRover
from SuperDrone import connect

import time

import serial.tools.list_ports
import sys

'''
Purpose of this test file is to ensure the functionally of the UAV under
the SuperDrone Class

This is a basic flight test in which the UAV will be autonomously:
1. arming and taking off to a height of 7 meters
2. traveling to one predesignated GPS coordinate
3. returning to start location and landing
'''


#Raspberry pi USB port connection string
connection_string = "/dev/ttyACM0"

print("Connecting to vehicle on:" , connection_string)

#create SuperRove instance
vehicle = connect(connection_string, wait_ready = True, baud = 57600)
print("Connected")

#ensure that vehicle information can be recieved
print("Vehicle Attributes:")
print(" System status: ")
vehicle.state.print_args()
print(" Global Location: %s" % vehicle.location_global)
print("Vehicle Mode: ", vehicle.mode)
print(" GPS: ", vehicle.gps_info)
print(" Global Location: %s" % vehicle.location_global)

#arm SuperDrone instance

vehicle.arm()

#ensure that vehicle information can be recieved
print("Vehicle Attributes:")
print(" System status: ")
vehicle.state.print_args()
print(" GPS: ", vehicle.gps_info)
print(" Global Location: %s" % vehicle.location_global)

vehicle.quitRTL()


print("Vehicle Attributes:")
print(" System status: ")
vehicle.state.print_args()
print(" Global Location: %s" % vehicle.location_global)

