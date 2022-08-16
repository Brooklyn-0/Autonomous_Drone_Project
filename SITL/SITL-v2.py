# script for flying autonomous quadcopter at UCI IAS lab
# adapted from UAV-FT-Basic_STIL.py

# file structure:
#
# Hydra-UAV
#    |--- webcampics/
#    |--- yolov5/
#           |--- best.pt
#    |--- SITL-v2.py
#    |--- model_test.jpg
#    |--- SITL_dataset


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

import timg  # display image in terminal to do camera test

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
def detection(path):
    global model
    # lists to store results value
    target_lst = []
    stop_lst = []

    for image in glob.glob(path + '/*.jpg'):
        result = model(image)
        length = len(result.pandas().xyxy[0].values)
        # if detects anything in the image, add values to the lists
        if length:
            # only detects target
            if length == 1 and result.pandas().xyxy[0].values[0][5] == 1:
                target_lst.append(result.pandas().xyxy[0].values[0][4])
                stop_lst.append(0.0)

            # only detects stop
            elif length == 1 and result.pandas().xyxy[0].values[0][5] == 0:
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
def stop_percentage(path):
    global model
   

    #os.system('sudo fswebcam -r 1920x1080 --save %s/webcampics/img_percentage' % path)
   
    result = model(path)
    for value in result.pandas().xyxy[0].values:
        # detects stop=0
        if value[5] == 0 and value[4] >= 0.7:
            x = value[2] - value[0]
            y = value[3] - value[1]
            area = x * y
            percentage = area / (1920*1080)
            return percentage * 100
    print('Cannot detect stop!')
    return -1
        

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
    #path = os.getcwd()
    #os.system('sudo fswebcam -r 1920x1080 --save %s/webcampics/img_test' % path)
    im = timg.Renderer()
    #im.load_image_from_file('webcampics/img_test.jpg')
    im.load_image_from_file('model_test.jpg')
    im.resize(160, 90)
    im.render(timg.ASCIIMethod)
    #file = '%s/webcampics/img_test' % path
    #if os.path.isfile(file):
        #os.remove(file)
    print('Camera test completed.')
    print('*******************************')
    time.sleep(2)
    print()

    # model test
    print('*******************************')
    print('Testing model...')
    
    model = torch.hub.load('ultralytics/yolov5', 'custom', path='yolov5/best.pt')
    img = 'model_test.jpg'
    result = model(img)
    print(result.pandas().xyxy[0])
    print('Model test completed.')
    print('*******************************')
    time.sleep(2)
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
    time.sleep(2)
    print()


    # take off
    vehicle.arm_and_takeoff(takeoffHeight)

    print(" local Location: %s" % vehicle.location_local)

    # adjust the drone's direction until detects the target
    # turn 45 degrees each time. 45 * 8 = 360 completes a circle
    target_detected = False
    for i in range(8):
        # take 10 photos in 2 seconds
        # for j in range(10):
        #     # take a photo with the webcam and save to a folder
        #     os.system('sudo fswebcam -r 1920x1080 --save %s/webcampics/img%s' % (path, str(j)))
        #     time.sleep(0.2)

        # now have 10 photos saved. detect whether target exists
        print('detecting target...')
        target_detected = detection('SITL_dataset/start')[0]
        # for k in range(10):
        #     file = '%s/webcampics/img%s' % (path, str(k))
        #     if os.path.isfile(file):
        #         os.remove(file)
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

        stop_detected = detection('SITL_dataset/DS1')[1]
        while not stop_detected:
            print('stop not detected...')
            # fly for 1s and hover, wait for processing
            grid_origin = vehicle.location_global
            location = (0, 1)
            radius = 1
            # go_in_grid function needs to be modified in superdrone.py
            vehicle.go_in_grid(location, altitude=takeoffHeight, pos_precision=radius, center=grid_origin)
            #vehicle.stop()

            # for j in range(10):
            #     # take a photo with the webcam and save to a folder
            #     os.system('sudo fswebcam -r 1920x1080 --save %s/webcampics/img%s' % (path, str(j)))
            #     time.sleep(0.2)

            # now have 10 photos saved. detect whether stop exists
            stop_detected = detection('SITL_dataset/DS6')[1]
            # for k in range(10):
            #     file = '%s/webcampics/img%s' % (path, str(k))
            #     if os.path.isfile(file):
            #         os.remove(file)
            if stop_detected:
                print("Stop detected! Mission stage 2 starts...")
                break

        # second stage of mission, detected stop sign, fly at speed 0.5 m/s
        vehicle.set_groundspeed(0.5)
        print('groundspeed set to 0.5')
        # if the stop takes over 50% of the image, stop, bypass it
        percentage = stop_percentage('SITL_dataset/DS7-1/WIN_20220815_14_40_03_Pro.jpg')
        print('current percentage:', percentage)
        while percentage < 1.12 and percentage > 0:
            # fly for 1s and hover, wait for processing
            print('percentage of stop < 1.12')
            grid_origin = vehicle.location_global
            location = (0, 0.5)
            radius = 1
            vehicle.go_in_grid(location, altitude=takeoffHeight, pos_precision=radius, center=grid_origin)
            #vehicle.stop()
            percentage = stop_percentage('SITL_dataset/DS7-6/WIN_20220815_14_41_03_Pro.jpg')
            print('current percentage:', percentage)
            #file = '%s/webcampics/img_percentage' % path
            #if os.path.isfile(file):
                #os.remove(file)

        if percentage == -1:
            print('Mission aborted.')
            back_home()
        
        else:
            # now bypass the target
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

            print("MissionImpossible success!!!")
            back_home()



except Exception:
    traceback.print_exc()
    print("error occured - setting to RTL")
    vehicle.quitRTL()
    print("Landed - inspect UAV")
    vehicle.close()
    sitl.stop
