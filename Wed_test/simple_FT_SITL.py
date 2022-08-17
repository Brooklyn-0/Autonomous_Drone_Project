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


sitl = dronekit_sitl.start_default();

connection_string = sitl.connection_string()

#Dronekit-SITL connection string
#connection_string = "127.0.0.1:5760"
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


    for i in range(5):

        grid_origin = vehicle.location_global
        #(dNorth, dEast) in meters
        location_1 = (0, 1)
        radius = 1


        print("moving to location: {} meters due North from origin".format(1))
        vehicle.go_in_grid(location_1, pos_precision=radius, center=grid_origin)
        time.sleep(2)

    vehicle.set_groundspeed(0.5)

    for j in range(5):
        grid_origin = vehicle.location_global
        #(dNorth, dEast) in meters
        location_1 = (0, 0.5)
        radius = 1


        print("moving to location: {} meters due North from origin".format(0.5))
        vehicle.go_in_grid(location_1, pos_precision=radius, center=grid_origin)
        time.sleep(2)


    vehicle.set_yaw(90, True)
    grid_origin = vehicle.location_global
    location = (0, 3)
    radius = 1
    vehicle.go_in_grid(location, altitude=takeoffHeight, pos_precision=radius, center=grid_origin)

    vehicle.set_yaw(270, True)
    grid_origin = vehicle.location_global
    location = (0, 3)
    radius = 1
    vehicle.go_in_grid(location, altitude=takeoffHeight, pos_precision=radius, center=grid_origin)

    
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



    
    




