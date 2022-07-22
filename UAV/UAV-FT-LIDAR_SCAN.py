import flight_utils
import SuperDrone
from SuperDrone import connect
import time
import serial.tools.list_ports
import sys
import traceback
from LiDAR import livox_python
from odin_client import EdgeClient
'''
Purpose of this file:
User Designates a GPS coordinate for which the UAV will perform a LiDAR scan of.
LiDAR scan will be done using the either the fly_circle or fly_square SuperDrone function
1. livox_python object will start connect the LiDAR before takeoff
2. SuperDrone instance will arm and takeoff
3. livox_python object will begin recording
4. UAV travels to target location (location_0) using go_in_grid
5. Once UAV reaches target, fly_circle is executed to perform the scanning maneuvor around the target
6. UAV flies in a circle then returns to the center of the target
7. livox_python terminates recording and saves rosbag file within ws_livox folder
8. UAV enters RTL and returns back to start point, lands, then disarms
9. all livox_python subprocess terminate
'''


#JetsonNano USB port connection string
connection_string = "/dev/ttyACM0"

print("Connecting to vehicle on:" , connection_string)

#create SuperRover instance
vehicle = connect(connection_string, wait_ready = True, baud = 57600)
print("Connected")

try:
        ec = EdgeClient(vehicle=vehicle)
        ec.register_exit_function()
        ec.start()

        #ensure that vehicle information can be recieved
        print("Vehicle Attributes:")
        print(" System status: ")
        vehicle.state.print_args()
        print(" local Location: %s" % vehicle.location_local)
        print(" GPS: ", vehicle.gps_info)
        print(" Global Location: %s" % vehicle.location_global)

        start_lidar = livox_python()

        #arm SuperDrone instance and set mode to Guided

        grid_origin = vehicle.location_global
        target_altitude = 5
        vehicle.set_groundspeed(1)

        vehicle.arm_and_takeoff(target_altitude)

        start_lidar.connect_lidar()
        time.sleep(2)
        print("LiDAR is starting to record")
        start_lidar.start_record()

        posDev = 1
        location_0 = (0, 3)  # location_0 is the target's location

        # print("moving to target location")
        # vehicle.go_in_grid(location_0, altitude=target_altitude, pos_precision=posDev, center=grid_origin, speed=1)

        # time.sleep(2)
        # print("executing fly_circle")
        # flightRadius = 6
        # vehicle.fly_circle(6, altitude=target_altitude, pos_precision=posDev, speed=1)

        #print("executing 45 Degrees ")
        #vehicle.scanTarget_45Degrees(target=location_0, center=grid_origin, altitude=target_altitude, pos_precision=posDev, speed=1)

        time.sleep(40)

        start_lidar.end_record()
        print("LiDAR recording ended")
        start_lidar.disconnect_lidar()
        ec.end_threads()
        
        print("returning to start_location and landing")
        vehicle.quitRTL()

        print("Vehicle Attributes:")
        print(" System status: ")
        vehicle.state.print_args()
        print(" Global Location: %s" % vehicle.location_global)

        start_lidar.terminate_all()
        print("All LiDAR processes terminated")
except Exception:
	print("error occured during flight:")
	traceback.print_exc()
	print("error occured - setting to RTL")
finally:
        vehicle.quitRTL()
        print("Landing - inspect UAV")
        #vehicle.close()
