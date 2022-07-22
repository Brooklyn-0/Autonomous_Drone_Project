from dronekit import Vehicle
from dronekit.util import errprinter
from pymavlink import mavutil, mavwp
from dronekit.mavlink import MAVConnection
from dronekit import connect as dkconnect
from dronekit import VehicleMode
from threading import Lock
from flight_utils import get_location_meters
from flight_utils import get_distance_meters, get_ground_distance_meters
import time
import math

class State:
    """
    Dynamically tracks state information in a threadsafe environment.
    
    :param kargs: Optional dictionary of attributes. Attributes can be dynamically added later
    """
    def __init__(self, **kargs):
        self.mod_state_lock = Lock()
        getter_str = "def get_{0}(self):\n\treturn self._{0}\nState.get_{0}=get_{0}"
        setter_str = "def set_{0}(self, value):\n\ttmp=self._{0}\n\tself.mod_state_lock.acquire()\n\tself._{0}=value\n\t" \
                     "self.mod_state_lock.release()\n\treturn tmp\nState.set_{0}=set_{0}"
        for e in kargs.keys():
            print("key", e)
            setattr(self, "_"+e, kargs[e])
            exec (getter_str.format(e))
            exec (setter_str.format(e))


    def print_args(self):
        """
        Prints every stored attribute and their corresponding values
        """
        for e in self.__dict__:
            print(e, getattr(self, e))


class SuperDrone(Vehicle):
    """
    A Vehicle from DroneKit with augmented capabilities and acessor methods.
    """

    @property
    def state(self):
        """
        Stores the drone state and attributes
        """
        return self._state

    def gps_info(self):
        return self.gps_0


    @property
    def location_global(self):
        """
        Convenience variable for accessing drone location relative to the global frame
        """
        return self.location.global_relative_frame


    @property
    def location_local(self):
        """
        Convenience variable for accessing drone location relative to home, or the
        start position of the drone.
        """
        return self.location.local_frame


    def _initstate(self):
        """
        Initializes the vehicle states to N/A
        """
        
        # Create the state and timestamp tracking objects
        self._state = State()
        self._state.accel = State()
        self._state.magno = State()
        self._state.gyro = State()
        self._state.barom = State()
        self._state.battery = State()
        self._state.sys = State()
        self._state.attitude = State()
        
        # initialize start time (to avoid errors)
        self._state.sys.time = -1
        self._state.sys.utime = -1

        #IMU
        self._state.accel.x = "N/A"
        self._state.accel.y = "N/A"
        self._state.accel.z = "N/A"

        self._state.gyro.x = "N/A"
        self._state.gyro.y = "N/A"
        self._state.gyro.z = "N/A"

        self._state.magno.x = "N/A"
        self._state.magno.y = "N/A"
        self._state.magno.z = "N/A"
        
        self._state.accel.timestamp = "N/A"
        self._state.gyro.timestamp = "N/A"
        self._state.magno.timestamp = "N/A"
        
        self._state.barom.press_abs = "N/A"
        self._state.barom.press_diff = "N/A"
        self._state.barom.temp = "N/A"
        self._state.barom.timestamp = "N/A"

        #battery info
        self._state.battery.temperature = "N/A"
        self._state.battery.voltages = "N/A"
        self._state.battery.current_battery = "N/A"
        self._state.battery.current_consumed = "N/A"
        self._state.battery.battery_remaining = "N/A"
        self._state.battery.time_remaining = "N/A"
        self._state.battery.charge_state = "N/A"
        self._state.battery.timestamp = "N/A"
                
        #sys info
        self._state.sys.drop_rate_comm = "N/A"
        self._state.sys.errors_count1 = "N/A"
        self._state.sys.errors_count2 = "N/A"
        self._state.sys.errors_count3 = "N/A"
        self._state.sys.load = "N/A"
        self._state.sys.timestamp = "N/A"


    def __init__(self, *args):
        super(SuperDrone, self).__init__(*args)
    
        #initialize the states to default values
        self._initstate()

        # Listen to RAW_IMU message for gyro, mango, and accel data
        @self.on_message('SCALED_IMU2')
        def listener(self, name, message):
            """
            The listener is called for messages that contain the string specified in the decorator,
            passing the vehicle, message name, and the message.
            
            The listener writes the message to the (newly attached) ``vehicle.raw_imu`` object 
            and notifies observers.
            """
            #store accelerometer values
            self._state.accel.x = message.xacc
            self._state.accel.y = message.yacc
            self._state.accel.z = message.zacc

            self._state.gyro.x = message.xgyro
            self._state.gyro.y = message.ygyro
            self._state.gyro.z = message.zgyro

            self._state.magno.x = message.xmag
            self._state.magno.y = message.ymag
            self._state.magno.z = message.zmag
            
            #store timestamps for IMU
            self._state.accel.timestamp = message.time_boot_ms    
            self._state.gyro.timestamp = message.time_boot_ms    
            self._state.magno.timestamp = message.time_boot_ms
            
            # Notify all observers of new message (with new value)
            #   Note that argument `cache=False` by default so listeners
            #   are updated with every new message
            self.notify_attribute_listeners('raw_imu', self) 

        # Create a message listener for the Barometer   
        @self.on_message('SCALED_PRESSURE')
        def listener(self, name, message):
            
            # store sensor values
            self._state.barom.press_abs = message.press_abs
            self._state.barom.press_diff = message.press_diff
            self._state.barom.temp = message.temperature
            
            # store timestamp
            self._state.barom.timestamp = message.time_boot_ms

           
        # Create a message listener for system updates
        @self.on_message('SYS_STATUS')
        def listener(self, name, message):
            self._state.sys.drop_rate_comm = message.drop_rate_comm
            self._state.sys.errors_count1 = message.errors_count1
            self._state.sys.errors_count2 = message.errors_count2
            self._state.sys.errors_count3 = message.errors_count3
            self._state.sys.load = message.load
            self._state.sys.timestamp = self._state.sys.time

            # store sensor values
            self._state.battery.temperature = None
            self._state.battery.voltages = message.voltage_battery
            self._state.battery.current_battery = message.current_battery
            self._state.battery.current_consumed = None
            self._state.battery.battery_remaining = message.battery_remaining
            self._state.battery.time_remaining = None
            self._state.battery.charge_state = None
            
            # store timestamp
            self._state.battery.timestamp = self._state.sys.time    
            
            
        # Create a message listener for system time updates
        @self.on_message('SYSTEM_TIME')
        def listener(self, name, message):
            self._state.sys.time = message.time_boot_ms
            self._state.sys.utime = message.time_unix_usec

    def set_groundspeed(self, new_speed):
        self.groundspeed = new_speed
            
    def go_in_grid(self, position, altitude=None, pos_precision=1, time_precision=0.5, center=None, speed=None):
        """
        Move the vehicle in a grid with origin -center-, oriented as NE. Altitude is constant.
        Positions are expressed in meters.

        :param altitude: altitude of scan
        :param time_precision: time between each distance check to end movement
        :param pos_precision: tolerance of position accuracy for targegt
        :param position: next position to reach
        :param pos_precision: radius of the circle around the target position that is considered position reached.
        :param center: origin of the cartesian field. This is position (0,0)
        :param speed: set new speed
        """
        currentLocation = self.location.global_relative_frame
        targetLocation = get_location_meters(center, position[0], position[1], altitude)
        targetDistance = get_distance_meters(currentLocation, targetLocation)
        self.simple_goto(targetLocation)
        print("DEBUG: targetLocation: {}".format(targetLocation))
        print("DEBUG: targetDistance from currentLocation: {}".format(targetDistance))

        while self.mode.name == "GUIDED" : #Stop action if we are no longer in guided mode.
            remainingDistance = get_distance_meters(self.location.global_relative_frame, targetLocation)
            print("Distance to target: ", remainingDistance)
            if remainingDistance <= pos_precision: #Just below target, in case of undershoot.
                print("Reached target")
                break;
            time.sleep(time_precision)


    def fly_square(self, length, altitude = 10, pos_precision=1, time_precision=0.5, speed=1):
        """
        Move the in a square, assumes target location already reached. Altitude is constant.
        Positions are expressed in meters.

        :param position: next position to reach
        :param precision: radius of the circle around the target position that is considered position reached.
        :param center: origin of the cartesian field. This is position (0,0)
        :param speed: set new speed
        """
        center = self.location.global_relative_frame

        print("center of fly_square: ", center)
        bottom_left = self.to_ned(FrdVector(-length/2, -length/2, 0))
        top_left = self.to_ned(FrdVector(length/2, -length/2, 0))
        bottom_right = self.to_ned(FrdVector(-length/2, length/2, 0))
        top_right = self.to_ned(FrdVector(length/2, length/2, 0))
        forward = self.to_ned(FrdVector(length/2, 0, 0))
        backward = self.to_ned(FrdVector(-length/2, 0, 0))

        movements = [bottom_left, top_left, bottom_right, top_right, forward, backward]
        for m in movements:
            new_coord = (center[0] + m.north, center[1] + m.east)
            self.go_in_grid(new_coord, altitude, pos_precision, time_precision, center, speed)



    def fly_circle(self, radius, altitude = 10, pos_precision=None, time_precision=0.5, slices = 8, speed=1):
        """
        flys a circlular pattern with the current position as the center, assumes target location already reached. Altitude is constant.
        Positions are expressed in meters.

        :param position: next position to reach
        :param precision: radius of the circle around the target position that is considered position reached.
        :param center: origin of the cartesian field. This is position (0,0)
        :param speed: set new speed
        """
        center = self.location.global_relative_frame

        print("center of fly_circle: ", center)
        for i in range(0, slices+1):
            a = i/slices * 2 * math.pi
            x = radius * math.cos(a)
            y = radius * math.sin(a)
            #Travel in a circle in increments of 45 degrees
            print("%4.1f%% manuever circle: moving (%4.2f North, %4.2f East, %d High, %5.2f degrees) from center" % (i/slices*100, x, y, altitude, a * 180/math.pi))
            self.go_in_grid((x, y), altitude, pos_precision, time_precision, center, speed)

        #Travel back to the center of the
        print("Traveling back to center of circle")
        self.go_in_grid((0, 0), altitude, pos_precision, time_precision, center, speed)


    def getHeading(self, currPos: location_global, targetPos: location_global) -> float:
        '''
        currPos = location global object of current UAV position
        targetPos = location global object of target location
        Bearing from point A to B, can be calculated as,

        β = atan2(X,Y),

        where, X and Y are two quantities and can be calculated as:

        X = cos θb * sin ∆L
        Y = cos θa * sin θb – sin θa * cos θb * cos ∆L

        returns the bearing in degrees

        deltaLat = targetPos.lat - currPos.lat

        radianDeltaL = math.radians(deltaLat)
        radianA = math.radians(currPos.lon)
        radianB = math.radians(targetPos.lon)

        X = math.cos(radianB) * math.sin(radianDeltaL)
        Y = (math.cos(radianA) * math.sin(radianB) ) - (math.sin(radianA) * math.cos(radianB) * math.cos(radianDeltaL))
        return math.degrees(math.atan2(Y, X))

        '''
        off_x = targetPos.lon - currPos.lon
        off_y = targetPos.lat - currPos.lat
        bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
        if bearing < 0:
            bearing += 360.00
        return bearing;

    def scanTarget_45Degrees(self, target, center, altitude = 10, pos_precision=None, time_precision=0.5, speed=None):
        """
                Move the vehicle to a position altitude meters away from the target location for 45 degree LiDAR scanning,
                    oriented as NE. Altitude is constant.

                target is the target position are expressed in meters (North, East) away from the center coordinate.

                :param altitude: altitude of scan
                :param time_precision: time between each distance check to end movement
                :param pos_precision: tolerance of position accuracy for target
                :param target: GPS coordinate for designated scanning (in meters north and east of center param)
                :param pos_precision: radius of the circle around the target position that is considered position reached.
                :param center: origin of the cartesian field. This is position (0,0)
                :param speed: set new speed

                for absolute yaw: 0 => N, 180 => S, E => 90, W => 270
        """
        # ensure you are at the correct altitude for scanning:
        print("Beginning scanTarget_45Degrees flight maneuver")
        print("0%%: Moving to altitude ", altitude)
        current_altitude = self.location_global.alt
        if current_altitude < altitude - pos_precision or current_altitude > altitude + pos_precision:
            print("adjusting altitude")
            self.go_in_grid((0, 0), altitude, pos_precision, time_precision, center, speed)

        print("25%%: Current yaw: ", math.degrees(self.attitude.yaw))


        #adjust the yaw of the UAV to point to the target
        UAV_currGPSPos = self.location.global_relative_frame
        targetGPSPos= get_location_meters(center, target[0], target[1], altitude)

        #adjusting the heading of the UAV to point at the targetLocation
        heading = self.getHeading(UAV_currGPSPos, targetGPSPos)
        print("50%%: Target is absolute %3.3f from UAV....moving yaw" % heading)
        self.set_yaw(heading)
        time.sleep(5)
        print("Current yaw: ", math.degrees(self.attitude.yaw))

        targetGroundDistance = get_ground_distance_meters(UAV_currGPSPos, targetGPSPos)

        print("DEBUG: targetGPSPos: {}".format(targetGPSPos))
        print("DEBUG: targetGroundDistance from currentLocation: {}".format(targetGroundDistance))

        '''
        two cases:
            is the UAV within altitude number of meters from the target <=
            is the UAV more than altitude number of meters from the target
        '''

        if targetGroundDistance >= altitude - pos_precision and targetGroundDistance <= altitude + pos_precision:
            print("100%%: UAV is already in acceptable position for scanning - scanning_45Degree")
            return

        elif targetGroundDistance - pos_precision > altitude:
            print("moving forwards to target radius")
            #travel targetGroundDistance - altitude forwards
            timeofTravel = float((targetGroundDistance - altitude)/speed)

            #move forwards at a velocity of speed and a duration of timeofTravel
            self.move(speed, 0,0, duration=timeofTravel)

        else:
            print("moving backwards to target radius")
            # travel altitude - targetGroundDistance backwards
            timeofTravel = float((altitude - targetGroundDistance) / speed)

            # move backwards at a velocity of speed and a duration of timeofTravel
            self.move(-speed, 0, 0, duration=timeofTravel)

        remainingDistance = get_ground_distance_meters(self.location.global_relative_frame, targetGPSPos)
        print("Ground Distance to target: ", remainingDistance)
        time.sleep(1)
        print("100%%: UAV is in acceptable position for scanning - scanning_45Degree")

    
    def go(self, vector, duration):
        """
        Convenience method to set vehicle velocity given a NedVector or a FrdVector

        :param vector: FrdVector or NedVector to move with
        :param duration: sleeps for a time which defines the distance that will be travelled
        """

        if isinstance(vector, FrdVector):#convert to NED frame if necessary
            vector = self.to_ned(vector)
        if isinstance(vector, NedVector):
            self.go_ned(vector.north, vector.east, vector.down, duration)
        else:
            raise ValueError("Invalid call to .go(): vector is neither a FrdVector nor a NedVector!")


    def stop(self):
        """
        Sets the drone velocity to zero. 
        Note: this method does not change modes or initiate landing sequence,
        nor does it gaurantee the stop message is respected
        """
        self.ml_ned(0,0,0);


    def ml_ned(self, north, east, down):
        """
        Sends a SINGLE mavlink message velocity command in the NED coordinate system.

        :param north: Northern velocity in NED coordinate system
        :param east: Eastern velocity in NED coordinate system
        :param down: Downward velocity in NED coordinate system
        """
        msg = self.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions (not used)
            north, east, down, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle

        self.send_mavlink(msg)
        

    def go_ned(self, north, east, down, duration=1):
        """
        Sets vehicle velocity in the NED coordinate system

        :param north: Northern velocity in NED coordinate system
        :param east: Eastern velocity in NED coordinate system
        :param down: Downward velocity in NED coordinate system
        :param duration: sleeps for a time which defines the distance that will be travelled
        """
        # send command to vehicle on 1 Hz cycle
        for x in range(0,int(duration*10)):
            self.ml_ned(north, east, down)
            print("Moving %3.2fN %3.2fE, %3.2f Down ... %3d has passed" % (north, east, down, x))
            time.sleep(0.1)
        return

    def move(self, front, right, down, duration):
        print("Called move with duration {}".format(duration))
        self.go(FrdVector(front, right, down), duration)
    

    def go_frd(self, front, right, down, duration=1):
        """
        Sets the vehicle velocity in the FRD coordinate system

        :param front: Northern velocity in NED coordinate system
        :param right: Eastern velocity in NED coordinate system
        :param down: Downward velocity in NED coordinate system
        :param duration: sleeps for a time which defines the distance that will be travelled
        """
        #TODO: Finish method
        yaw = self.attitude.yaw
        ned_vector = to_ned(front, right, down)

        self.go_ned(north_velocity.north, ned_vector.east, ned_vector.down, duration)
        return

    def send_global_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors.
        This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only
        velocity components
        (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_global_int).

        Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
        with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
        velocity persists until it is canceled. The code below should work on either version
        (sending the message multiple times does not cause problems).

        See the above link for information on the type_mask (0=enable, 1=ignore).
        At time of writing, acceleration and yaw bits are ignored.
        """
        msg = self.message_factory.set_position_target_global_int_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0,  # lat_int - X Position in WGS84 frame in 1e7 * meters
            0,  # lon_int - Y Position in WGS84 frame in 1e7 * meters
            0,  # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
            # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
            velocity_x,  # X velocity in NED frame in m/s
            velocity_y,  # Y velocity in NED frame in m/s
            velocity_z,  # Z velocity in NED frame in m/s
            0, 0, 0,  # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

        # send command to vehicle on 1 Hz cycle
        for x in range(0, duration):
            self.send_mavlink(msg)
            time.sleep(1)


    def set_yaw(self, heading, relative=False):
        """
        Turns the drone to the given heading

        :param heading: Heading to turn to, in degreees
        :param relative: If True, yaw is relative to direction of travel

        Return:
            A NedVector with the corresponding values to the FRD values
        """
        #must add to make yawing work
        self.send_global_velocity(0,0,0,1)

        if relative:
            is_relative=1 #yaw relative to direction of travel
        else:
            is_relative=0 #yaw is an absolute angle
        # create the CONDITION_YAW command using command_long_encode()
        msg = self.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
            0, #confirmation
            abs(heading),    # param 1, yaw in degrees
            0,          # param 2, yaw speed deg/s
            math.copysign(1, heading),          # param 3, direction -1 ccw, 1 cw
            is_relative, # param 4, relative offset 1, absolute angle 0
            0, 0, 0)    # param 5 ~ 7 not used
            
        # send command to vehicle
        self.send_mavlink(msg)


    def to_ned(self, vector):
        """
        Translated coordinates from FRD (front right down) to NED (north east down)
        axis system

        :param vector: FrdVector to convert to NED frame of reference

        :returns: A NedVector with the corresponding values to the FRD values
        """
        if not isinstance(vector, FrdVector):
            raise ValueError("Invalid vector argument! Must be a FrdVector instance")
        yaw = self.attitude.yaw

        north_velocity = vector.front*math.cos(yaw) - vector.right*math.sin(yaw)
        east_velocity = vector.front*math.sin(yaw) + vector.right*math.cos(yaw)
        
        return NedVector(north_velocity, east_velocity, vector.down)


    def to_frd(self, north, east, down):
        """
        Translated cordinates from NED (north east down) to FRD (front right down)
        axis system

        :param north: Northern direction in NED coordinate system
        :param east: Eastern direction in NED coordinate system
        :param down: Downward direction in NED coordinate system

        :returns: A FrdVector with the corresponding valued to the NED vector
        """
        #TODO: Finish method
        #NOT SURE IF NECESSARY
        raise NotImplementedError()


    def arm(self):
        """
        Arms vehicle and waits for arming confirmation
        note: Vehicle will automaticaly disarm in ~3 seconds if still on the ground
        """
        
        print("Basic pre-arm checks")
        # Don't try to arm until autopilot is ready
        while not self.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)
            self.mode = VehicleMode("GUIDED")

        # Copter should arm in GUIDED mode
        print("Entering guided mode")
        while(self.mode != VehicleMode("GUIDED")):
            self.mode    = VehicleMode("GUIDED")
            print("waiting to enter guided mode - still not guided")
            time.sleep(3);
            
        print("Arming motors")
        self.armed   = True

        # Confirm vehicle armed before attempting to take off
        while not self.armed:
            print(" Waiting for arming...")
            time.sleep(3)
            if not self.armed:
                self.armed = True


    def arm_and_takeoff(self, aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        
        :param aTargetAltitude: Target altitude in meters
        """
        self.arm()
        print("Taking off!")
        self.simple_takeoff(aTargetAltitude) # Take off to target altitude
        self.starting_location = self.location.global_relative_frame

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print(" Altitude: ", self.location.global_relative_frame.alt)
            #Break and return from function just below target altitude.
            if self.location.global_relative_frame.alt>=aTargetAltitude*0.95:
                print("Reached target altitude")
                break
            time.sleep(1)


    def quitRTL(self):
        """
        Puts the vehicle in RTL mode and waits for landing.
        """
        print("Returning to launch")
        self.go(FrdVector(0,0,-1), 1)
        self.mode = VehicleMode('RTL')

        while(self.mode != VehicleMode('RTL')):
           self.mode = VehicleMode('RTL')

        print(" Mode is RTL!")
        while(self.armed):
          print(" Waiting for land...")
          time.sleep(2)
        print(" Landed!")


    def quit_manual_RTL(self):
      self.go(FrdVector(0,0,-1), 1)
      self.mode = VehicleMode("LAND")
      self.starting_location.altitude = 10
      self.simple_goto(self.starting_location, groundspeed=4)

      while(self.mode != VehicleMode("LAND")):
        self.mode = VehicleMode("LAND")
      print("Mode is LAND")
      while self.armed:
        print("Waiting land - drone still armed")
        time.sleep(2)
      print("Landed")

class NedVector():
    """
    A Vector in the NED (north east down) coordinate system.
    
    :param north: Value of the vector in the north direction
    :param east: Value of the vector in the east direction
    :param down: Value of the vector in the down direction
    """
    def __init__(self, north, east, down):
        self.north = north
        self.east = east
        self.down = down


class FrdVector():
    """
    A Vector in the FRD (front right down) coordinate system.
    
    :param front: Value of the vector in the front direction
    :param right: Value of the vector in the right direction
    :param down: Value of the vector in the down direction
    """
    def __init__(self, front, right, down):
        self.front = front
        self.right = right
        self.down = down


def connect(ip,
    _initialize=True,
    wait_ready=False,
    status_printer=errprinter,
    rate=10,
    baud=57600,
    heartbeat_timeout=30,
    source_system=255,
    use_native=False):
    """
    Convenience method adapted from dronekit source code. Connects to a SuperDrone with the given
    parameters.

    Returns a :py:class:`Vehicle` object connected to the address specified by string parameter ``ip``.
    The method is usually called with ``wait_ready=True`` to ensure that vehicle parameters and (most) attributes are
    available when ``connect()`` returns.

    :param String ip: IP for target address - e.g. 127.0.0.1:14550.
    :param Bool/Array wait_ready: If ``True`` wait until all default attributes have downloaded before the method returns
    :param status_printer: Method of signature ``def status_printer(txt)`` that prints status messages
    :param int rate: Data stream refresh rate. The default is 4Hz (4 updates per second).
    :param int baud: The baud rate for the connection. The default is 57600.
    :param int heartbeat_timeout: Connection timeout value in seconds (default is 30s).
        If a heartbeat is not detected within this time an exception will be raised.
    :param int source_system: The MAVLink ID of the :py:class:`Vehicle` object returned by this method (by default 255).
    :param bool use_native: Use precompiled MAVLink parser.

    :returns: A connected SuperDrone vehicle
    """

    vehicle_class = SuperDrone

    handler = MAVConnection(ip, baud=baud, source_system=source_system, use_native=use_native)
    vehicle = vehicle_class(handler)

    if _initialize:
        vehicle.initialize(rate=rate, heartbeat_timeout=heartbeat_timeout)

    if wait_ready:
        if wait_ready == True:
            vehicle.wait_ready(True)
        else:
            vehicle.wait_ready(*wait_ready)

    return vehicle

