import StartLidar_New
import SuperDrone 
import time
import traceback

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

try:

	#ensure that vehicle information can be recieved
	print("Vehicle Attributes:")
	print(" System status: ")
	vehicle.state.print_args()
	print(" GPS: ", vehicle.gps_info)
	print(" Global Location: %s" % vehicle.location_global)


	#arm SuperDrone instance and set mode to Guided and takeoff at takeoffHeight (meters)
	takeoffHeight = 7

	#groundspeed is set in m/s
	vehicle.set_groundspeed(1)

	vehicle.arm_and_takeoff(takeoffHeight)

	print(" local Location: %s" % vehicle.location_local)


	#move in delta North and delta east coordinates in meters, origin will be the bootup 		location of the rover

	grid_origin = vehicle.location_global



	#(dNorth, dEast) in meters
	location_1 = (0, 7)
	radius = 1


	print("moving to location_1: {} meters due North from origin".format(3))
	vehicle.go_in_grid(location_1, pos_precision=radius, center=grid_origin)

	print("location 1 arrived")


	# print("moving to location_3: {} meters due East from origin".format(x))
	# vehicle.go_in_grid(location_3,  pos_precision=radius, center=grid_origin)

	# print("Location 3 arrived")

	
	print("returning to start_location and landing")

	vehicle.quitRTL()

	print("Returned and Landed")
	print("Vehicle Attributes:")
	print(" System status: ")
	vehicle.state.print_args()
	print(" Global Location: %s" % vehicle.location_global)
	

except Exception:
	traceback.print_exc()
	print("error occured - setting to RTL")
	vehicle.quitRTL()
	print("Landing - inspect UAV")



	
	




