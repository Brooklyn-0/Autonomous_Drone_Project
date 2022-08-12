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
import os
import serial.tools.list_ports
import sys

# return and land
def back_home():
	print("returning to start_location and landing")
	vehicle.quitRTL()
	print("Returned and Landed")
	print("Vehicle Attributes:")
	print(" System status: ")
	vehicle.state.print_args()
	print(" Global Location: %s" % vehicle.location_global)
	vehicle.close()
	sitl.stop

# let the target in the center of the camera
def direction_adjustment_target():
	pass
# let the stop sign in the center
def direction_adjustment_stop():
	pass

# detect whether the target is in the image and its probability, return true
# if more than half probabilty > 0.7, return false otherwise
def detect_target():


# detect stop sign, following similar rules as above
def detect_stop():


# use ToF sensor as a failsafe, return the distance data.
def ToF_failsafe():


# return the percentage of target in the image
def target_percentage():




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


	# get current working directory
	path = os.getcwd()
	# adjust the drone's direction until detects the target
	# turn 45 degrees each time. 45 * 8 = 360 completes a circle
	target_detected = False
	for i in range(8):
		# take 10 photos in 2 seconds
		for j in range(10):
			# take a photo with the webcam and save to a folder
			os.system('sudo fswebcam -r 1980x1080 --save %s/webcampics/img%s' % (path, str(j)))
			time.sleep(0.2)

		# now have 10 photos saved. detect whether target exists
		target_detected = detect_target()
		for k in range(10):
			file = '%s/webcampics/img%s' % (path, str(k))
			if os.path.isfile(file):
				os.remove(file)
		if target_detected:
			print('Target detected! Mission stage 1 starts...')
			break
		# turn 45 degrees to the left
		vehicle.set_yaw(315, True)
		

	if not target_detected:
		print("No target in sight. Mission aborted.")
		back_home()

	else:
		# start flying towards the target
		# how to get and analyze sensor data continuously in this script?
		# assuming we have a failsafe already

		# how to set waypoint and use simple_goto?
		# assmue we've set a waypoint

		# first stage if mission, at speed 1m/s
		# use a while loop, quit when detect the stop sign
		
		#direction_adjustment_target()
		stop_detected = detect_stop()
		while not stop_detected:
			# fly for 1s and hover, wait for processing
			grid_origin = vehicle.location_global
			location = (0, 10)
			radius = 1
			vehicle.go_in_grid(location, altitude=takeoffHeight, pos_precision=radius, center=grid_origin)
			time.sleep(1)
			vehicle.stop()
			#direction_adjustment_target()

			for j in range(10):
				# take a photo with the webcam and save to a folder
				os.system('sudo fswebcam -r 1980x1080 --save %s/webcampics/img%s' % (path, str(j)))
				time.sleep(0.2)

			# now have 10 photos saved. detect whether stop exists
			stop_detected = detect_stop()
			for k in range(10):
				file = '%s/webcampics/img%s' % (path, str(k))
				if os.path.isfile(file):
					os.remove(file)
			if stop_detected:
				print("Stop detected! Mission stage 2 starts...")
				break
			
		# second stage of mission, detected stop sign, fly at speed 0.5 m/s
		vehicle.set_groundspeed(0.5)
		# if the target takes over 50% of the image, stop, bypass it
		percetage = target_percentage()
		while percetage < 0.5:
			# fly for 1s and hover, wait for processing
			grid_origin = vehicle.location_global
			location = (0, 10)
			radius = 1
			vehicle.go_in_grid(location, altitude=takeoffHeight, pos_precision=radius, center=grid_origin)
			time.sleep(1)
			vehicle.stop()
			percetage = target_percentage()
			time.sleep(1)

		# now by pass the target
			

		print("MissionImpossible success!!!")
		back_home()
	
	

except Exception:
	traceback.print_exc()
	print("error occured - setting to RTL")
	vehicle.quitRTL()
	print("Landed - inspect UAV")
	vehicle.close()
	sitl.stop
