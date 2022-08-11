# script for flying autonomous quadcopter at UCI IAS lab
# adapted from UAV-FT-Basic_STIL.py

from LiDAR import livox_python
import SuperDrone 
import time
import traceback
import dronekit_sitl

#for the dronekit imports necessary to run the utilities of SuperRover
from SuperDrone import connect

import time

import serial.tools.list_ports
import sys


# let the target in the center of the camera
def direction_adjustment_target():

# let the stop sign in the center
def direction_adjustment_stop():



# detect whether the target is in the image and its probability, return true
# if more than half probabilty > 0.7, return false otherwise
def detect_target():


# detect stop sign, following similar rules as above
def detect_stop():


# use ToF sensor as a failsafe. stop and hover if anything within 1 m.
def ToF_failsafe():


# connect to drone and take off

sitl = dronekit_sitl.start_default();

connection_string = sitl.connection_string()

#Dronekit-SITL connection string
connection_string = "udp:127.0.0.1:14551"

print("Connecting to vehicle on:" , connection_string)

#create SuperDrone instance
vehicle = connect(connection_string, wait_ready = True)
print("Connected")

try:

	#ensure that vehicle information can be recieved
	print("Vehicle Attributes:")
	print(" System status: ")
	vehicle.state.print_args()
	print(" GPS: ", vehicle.gps_info)
	print(" Global Location: %s" % vehicle.location_global)


	#arm SuperDrone instance and set mode to Guided and takeoff at takeoffHeight (meters)
	takeoffHeight = 1.25

	#groundspeed is set in m/s
	vehicle.set_groundspeed(1)

	vehicle.arm_and_takeoff(takeoffHeight)

	print(" local Location: %s" % vehicle.location_local)


	# adjust the drone's direction until detects the target
	# turn 45 degrees each time. 45 * 8 = 360 completes a circle
	target_detected = False
	for i in range(8):
		# take 10 photos in 2 seconds
		for j in range(10):
			# take a photo with the webcam and save to a folder
			# ******
			# code here
			# ******
			time.sleep(0.2)

		# now have 10 photos saved. detect whether target exists
		target_detected = detect_target()
		if target_detected:
			break
		# turn 45 degrees to the left
		# ******
		# code here
		# ******

	if not target_detected:
		print("No target in sight. Mission aborted.")
		print("returning to start_location and landing")

		vehicle.quitRTL()

		print("Returned and Landed")
		print("Vehicle Attributes:")
		print(" System status: ")
		vehicle.state.print_args()
		print(" Global Location: %s" % vehicle.location_global)
		vehicle.close()
		sitl.stop

	# start flying towards the target
	else:
		# how to get and analyze sensor data continuously in this script?
		# assuming we have a failsafe already

		# how to set waypoint and use simple_goto?
		# assmue we've set a waypoint

		# first stage if mission, at speed 1m/s
		# use a while loop, quit when detect the stop sign
		stop_detected = detect_stop()
		while not stop_detected:
			direction_adjustment_target()
			# fly for 1s and hover, wait for processing
			# ******
			# code here
			# ******
			for j in range(10):
				# take a photo with the webcam and save to a folder
				# ******
				# code here
				# ******
				time.sleep(0.2)

			# now have 10 photos saved. detect whether stop exists
			stop_detected = detect_stop()
			if stop_detected:
				break
			
		# second stage of mission, detected stop sign, fly at speed 0.5 m/s
		vehicle.set_groundspeed(0.5)
		# if the target takes up over 70% of the image, stop, bypass it





	
	print("MissionImpossible success!!!")
	print("returning to start_location and landing")

	vehicle.quitRTL()

	print("Returned and Landed")
	print("Vehicle Attributes:")
	print(" System status: ")
	vehicle.state.print_args()
	print(" Global Location: %s" % vehicle.location_global)
	vehicle.close()
	sitl.stop
	

except Exception:
	traceback.print_exc()
	print("error occured - setting to RTL")
	vehicle.quitRTL()
	print("Landed - inspect UAV")
	vehicle.close()
	sitl.stop
