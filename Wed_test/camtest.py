import StartLidar_New
import SuperDrone 
import time
import traceback

#for the dronekit imports necessary to run the utilities of SuperRover
from SuperDrone import connect

import time

import serial.tools.list_ports
import sys

import timg  # display image in terminal to do camera test

import torch    # yolov5 model
import glob

'''
Purpose of this test file is to ensure the functionally of the UAV under
the SuperDrone Class

This is a basic flight test in which the UAV will be autonomously:
1. arming and taking off to a height of 7 meters
2. traveling to one predesignated GPS coordinate
3. returning to start location and landing
'''


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
	takeoffHeight = 1.25

	#groundspeed is set in m/s
	vehicle.set_groundspeed(1)

	vehicle.arm_and_takeoff(takeoffHeight)

	print(" local Location: %s" % vehicle.location_local)


	# camera test
    print('*******************************')
    print('Testing camera...')
    path = os.getcwd()
    os.system('sudo fswebcam -r 640x640 --save %s/webcampics/test0/img_test' % path)
    time.sleep(3)
    im = timg.Renderer()
    im.load_image_from_file('webcampics/test0/img_test.jpg')
    im.resize(160, 90)
    im.render(timg.ASCIIMethod)
    
    print('Camera test completed.')
    print('*******************************')
    time.sleep(2)
    print()

    # model test
    print('*******************************')
    print('Testing model...')
    
    model = torch.hub.load('ultralytics/yolov5', 'custom', path='yolov5/best.pt')
    img = 'webcampics/test0/img_test.jpg'
    start = time.process_time()
    result = model(img)
    print('Processing the image takes:', time.process_time()-start)
    time.sleep(3)
    print()
    print(result.pandas().xyxy[0])
    print('Model test completed.')
    print('*******************************')
    time.sleep(2)
    print()

	for j in range(10):
	    # take a photo with the webcam and save to a folder
	    os.system('sudo fswebcam -r 640x640 --save %s/webcampics/test1/img%s' % (path, str(j)))
	    time.sleep(0.2)

	# now have 10 photos saved. detect whether target exists
    print('detecting target...')
    target_detected = detection('webcampics/test1')[0]
    if target_detected:
    	print('target detected')
    else:
    	print('No target detected')
    time.sleep(3)


	
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



	
