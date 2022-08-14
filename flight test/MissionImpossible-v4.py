# script for flying autonomous quadcopter at UCI IAS lab
# adapted from UAV-FT-Basic_STIL.py

# file structure:
#
# Hydra-UAV
#    |--- webcampics/
#    |--- yolov5/
#           |--- best.pt
#    |--- MissionImpossible.py
#    |--- model_test.jpg
#


# drone libraries
from LiDAR import livox_python
import SuperDrone
import time
import traceback
import dronekit_sitl
from SuperDrone import connect
import time
import os
import serial.tools.list_ports
import sys

import board  # modules to run the sensor
import adafruit_vl53l4cd

import timg  # display image in terminal to do camera test

import threading  # use multithreading to get sensor data into our script

import torch    # yolov5 model
import glob

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


# detect whether the target/stop is in the image and its probability
# return true if >= 7 images detects target with probabilty >= 0.85
# return true if >= 7 images detects stop with probability >= 0.7
# return two booleans [target, stop] to indicate whether detects these two objects
# note: the numpy array prints target/stop in the order of higher confidence level
def detection():
    global model
    # lists to store results value
    target_lst = []
    stop_lst = []

    for image in glob.glob('webcampics/*.jpg'):
        result = model(image)
        length = len(result.pandas().xyxy[0].values)
        # if detects anything in the image, add values to the lists
        if length:
            # only detects target
            if length == 1 && result.pandas().xyxy[0].values[0][5] == 1:
                target_lst.append(result.pandas().xyxy[0].values[0][4])
                stop_lst.append(0.0)

            # only detects stop
            elif length == 1 && result.pandas().xyxy[0].values[0][5] == 0:
                stop_lst.append(result.pandas().xyxy[0].values[0][4])
                target_lst.append(0.0)

            # detects both
            else:
                for value in result.pandas().xyxy[0].values:
                    # detects target=1
                    if value[5]:
                        target_lst.append(value[4])                    
                    # detects stop=0
                    else:
                        stop_lst.append(value[4])
                    
        # no detection
        else:
            target_lst.append(0.0)
            stop_lst.append(0.0)

    # determine whether detects target/stop
    target = len([i for i in target_lst if i >= 0.85 ]) >= 7
    stop = len([i for i in stop_lst if i >= 0.7 ]) >= 7

    return [target, stop]


# return the percentage of stop in the image
def stop_percentage():
    global model
    global path

    os.system('sudo fswebcam -r 1920x1080 --save %s/webcampics/img_percentage' % path)
    img = 'webcampics/img_percentage.jpg'
    result = model(img)
    for value in result.pandas().xyxy[0].values:
        # detects stop=0
        if value[5] == 0 && value[4] >= 0.7:
            x = value[2] - value[0]
            y = value[3] - value[1]
            area = x * y
            percentage = area / (1920*1080)
            return percetage * 100
    print('Cannot detect stop!')
    return -1
        


# use ToF sensor as a failsafe, return the distance data.
def ToF_failsafe():


# connect to drone
sitl = dronekit_sitl.start_default();

connection_string = sitl.connection_string()

# Dronekit-SITL connection string
connection_string = "udp:127.0.0.1:14551"

print("Connecting to vehicle on:", connection_string)

# create SuperDrone instance
vehicle = connect(connection_string, wait_ready=True)
print("Connected")

try:

    print('Testing camera, sensor, and yolov5 model...')  # order: camera, sensor(1), model, sensor(2)

    # camera test
    print('*******************************')
    print('Testing camera...')
    path = os.getcwd()
    os.system('sudo fswebcam -r 1920x1080 --save %s/webcampics/img_test' % path)
    im = timg.Renderer()
    im.load_image_from_file('webcampics/img_test.jpg')
    im.resize(160, 90)
    im.render(timg.ASCIIMethod)
    file = '%s/webcampics/img_test' % path
    if os.path.isfile(file):
        os.remove(file)
    print('Camera test completed.')
    print('*******************************')
    time.sleep(1)
    print()

    # sensor test1
    print('*******************************')
    print('Testing sensor(1)...')
    print('Put something in front of the sensor in 1 seconds')
    time.sleep(1)
    print('Initializing the sensor...')

    i2c = board.I2C()

    vl53 = adafruit_vl53l4cd.VL53L4CD(i2c)

    # OPTIONAL: can set non-default values
    vl53.inter_measurement = 50
    vl53.timing_budget = 50

    print("VL53L4CD Simple Test.")
    print("--------------------")
    model_id, module_type = vl53.model_info
    print("Model ID: 0x{:0X}".format(model_id))
    print("Module Type: 0x{:0X}".format(module_type))
    print("Timing Budget: {}".format(vl53.timing_budget))
    print("Inter-Measurement: {}".format(vl53.inter_measurement))
    print("--------------------")

    vl53.start_ranging()

    for i in range(10):
        while not vl53.data_ready:
            pass
        vl53.clear_interrupt()
        print("Distance: {} cm".format(vl53.distance))

    print('Sensor test1 completed.')
    print('*******************************')
    time.sleep(1)
    print()

    # model test
    print('*******************************')
    print('Testing model...')
    model = torch.hub.load('ultralytics/yolov5s', 'custom', path='yolov5/best.pt')
    img = 'model_test.jpg'
    result = model(img)
    print(result.pandas().xyxy[0])
    print('Model test completed.')
    print('*******************************')
    time.sleep(1)
    print()

    # sensor test2
    print('*******************************')
    print('Testing sensor(2)...')
    print('Put something in front of the sensor in 1 seconds')
    time.sleep(1)
    for i in range(10):
        while not vl53.data_ready:
            pass
        vl53.clear_interrupt()
        print("Distance: {} cm".format(vl53.distance))

    print('Sensor test2 completed.')
    print('*******************************')
    time.sleep(1)
    print()

    # ensure that vehicle information can be recieved
    print('Receiving drone information...')
    print("Vehicle Attributes:")
    print(" System status: ")
    vehicle.state.print_args()
    print(" GPS: ", vehicle.gps_info)
    print(" Global Location: %s" % vehicle.location_global)

    # arm SuperDrone instance and set mode to Guided and takeoff at takeoffHeight (meters)
    takeoffHeight = 1.25
    print('Takeoff height:', takeoffHeight)

    # groundspeed is set in m/s
    groundspeed = 1
    vehicle.set_groundspeed(groundspeed)
    print('Groundspeed:', groundspeed)
    print('Pretests all done. Ready to take off!')
    time.sleep(1)
    print()


    # take off
    vehicle.arm_and_takeoff(takeoffHeight)

    print(" local Location: %s" % vehicle.location_local)

    # adjust the drone's direction until detects the target
    # turn 45 degrees each time. 45 * 8 = 360 completes a circle
    target_detected = False
    for i in range(8):
        # take 10 photos in 2 seconds
        for j in range(10):
            # take a photo with the webcam and save to a folder
            os.system('sudo fswebcam -r 1920x1080 --save %s/webcampics/img%s' % (path, str(j)))
            time.sleep(0.2)

        # now have 10 photos saved. detect whether target exists
        target_detected = detect_target()[0]
        for k in range(10):
            file = '%s/webcampics/img%s' % (path, str(k))
            if os.path.isfile(file):
                os.remove(file)
        if target_detected:
            print('Target detected! Mission stage 1 starts...')
            break
        # turn 45 degrees to the left
        print('Fail to detect target at angle %s' % str(i*45))
        vehicle.set_yaw(315, True)

    if not target_detected:
        print("No target in sight. Mission aborted.")
        back_home()

    else:
        # start flying towards the target
        # first stage of mission at speed 1m/s
        # second stage of mission 0.5m/s

        stop_detected = detect_stop()[1]
        while not stop_detected:
            print('stop not detected...')
            # fly for 1s and hover, wait for processing
            grid_origin = vehicle.location_global
            location = (0, 10)
            radius = 1
            # go_in_grid function needs to be modified in superdrone.py
            vehicle.go_in_grid(location, altitude=takeoffHeight, pos_precision=radius, center=grid_origin)
            vehicle.stop()

            for j in range(10):
                # take a photo with the webcam and save to a folder
                os.system('sudo fswebcam -r 1920x1080 --save %s/webcampics/img%s' % (path, str(j)))
                time.sleep(0.2)

            # now have 10 photos saved. detect whether stop exists
            stop_detected = detect_stop()[1]
            for k in range(10):
                file = '%s/webcampics/img%s' % (path, str(k))
                if os.path.isfile(file):
                    os.remove(file)
            if stop_detected:
                print("Stop detected! Mission stage 2 starts...")
                break

        # second stage of mission, detected stop sign, fly at speed 0.5 m/s
        vehicle.set_groundspeed(0.5)
        # if the stop takes over 50% of the image, stop, bypass it
        percetage = stop_percentage()
        while percetage < 50 && percetage > 0:
            # fly for 1s and hover, wait for processing
            print('percentage of stop < 50')
            grid_origin = vehicle.location_global
            location = (0, 10)
            radius = 1
            vehicle.go_in_grid(location, altitude=takeoffHeight, pos_precision=radius, center=grid_origin)
            vehicle.stop()
            time.sleep(1)
            percetage = stop_percentage()

        if percetage == -1:
            print('Mission aborted.')
            back_home()
        
        else:
            # now bypass the target
            print("MissionImpossible success!!!")
            back_home()



except Exception:
    traceback.print_exc()
    print("error occured - setting to RTL")
    vehicle.quitRTL()
    print("Landed - inspect UAV")
    vehicle.close()
    sitl.stop
