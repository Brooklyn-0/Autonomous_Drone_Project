'''
Purpose of this file:
TCP manager for Jetson Nano to send LiDAR data to the ground computer
this should not physically slice or manipulate the data being sent, it should 
only send data that it has been given
'''

import csv
import atexit
from datetime import datetime
import socket
import pickle
import rospy
import os
from sensor_msgs.msg import PointCloud2
import sys
import threading
from threading import Thread
import time
import traceback

DEBUG=1
TESTING=1
ACK_DATA_SIZE = 13
GROUND_LAPTOP = "10.42.0.1"

def debug(line):
  if(DEBUG):
    print(line)

def log_to_file(data):
  f = open('log.txt', "wb")
  f.write(data)
  f.close()

bINTER = b"@!?*"  
bEND   = b"*?!@"

ACK_DATA_SIZE = 13

class EdgeClient(Thread):
    def __init__(self, ip='localhost', port=1028, vehicle=None):
      Thread.__init__(self)
      self.setDaemon = True
      self.folder_name = 'Flight_Logs'
      self.client_file = None
      self.csv_writer = None
      self.init_file()
      
      #set up TCP connection
      self.discoverable_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
      self.ip = ip
      self.port = port
      self.FRAME_SIZE = 900393
      self.send_frame_size = True
      server_address = (ip, port)

      self.discoverable_socket.connect(server_address)
      self.initialize_rospy()

      self.vehicle = vehicle 
      
      self.DONE = False

      # create a thread for receiving
      # self.timer_queue = []
      # self.lock = threading.Lock()
      # self.receive_thread = Thread(target=self.receive,
      #                                args=(self.timer_queue, self.DONE,))
      # self.receive_thread.setDaemon(True)

      self.ack_received = True
        
    def init_file(self):
      path = os.path.dirname(os.getcwd())
      path = path + '/' + self.folder_name
      if not os.path.exists(path):
          os.mkdir(path)
        
      file_name = datetime.now().strftime("%m_%d_%Y_%H-%M-%S") + '_client_log.csv'
      self.client_file = open(path + '/' + file_name, "a+")
      self.csv_writer = csv.writer(self.client_file)
      self.csv_writer.writerow(['seq', 'rtt'])

    def initialize_rospy(self):
        '''
        initialize_rospy utilizes ros to obtain the frames from the LiDAR by
        subscribing to the data it generates
        '''
        
        rospy.init_node('livox_client', anonymous=False)
        self.sub = rospy.Subscriber(
            "/livox/lidar", PointCloud2, queue_size=1, buff_size=2**24, callback=self.livox_callback)

    def livox_callback(self, frame) -> None:
        '''
        livox_callback is called once a frame is received from the LiDAR
        Serialize the data with delimieters and send to the server over
        the tcp connection

        :param frame: LiDAR frame
        '''

        # Make sure that ack has been received before livox_callback is called again
        if not self.ack_received or self.client_file.closed:
          return
        
        try:
          self.ack_received = False

          start_time = time.time()

          current_location = self.vehicle.location_global

          print(current_location.alt)
          data = b''
          data = (pickle.dumps(frame)
                  + bINTER
                  + pickle.dumps(current_location.alt)
                  + bEND)

          print("len data: ", len(data))

          self.discoverable_socket.send(data)
          debug("Sent data to server, awaiting ack")

          
          ack = self.discoverable_socket.recv(ACK_DATA_SIZE)
          while not ack:
            ack = self.discoverable_socket.recv(ACK_DATA_SIZE)
            
          end_time = time.time()

          ack = pickle.loads(ack)
          debug("ack: " + str(ack))
          self.ack_received = True

          self.csv_writer.writerow([frame.header.seq, 1000*(end_time - start_time), current_location])
        except IOError:
          print("IOError")
          pass
        except Exception as e:
          self.client_file.close()
          debug("Exception occurred")
          debug(e)
          traceback.print_exc()
          self.discoverable_socket.close()
          sys.exit()
          
    def register_exit_function(self):
      atexit.register(self.close_file_and_exit)

    def close_file_and_exit(self):
      print("closing file")
      self.client_file.close()
      sys.exit()

    def start_main_thread(self):
      self.start()
      
    def run(self):
      rospy.spin()
        
    def end_threads(self):
      print("endng threads") 
      self.DONE = True

      self.client_file.close()
      
      self.discoverable_socket.close()
      rospy.signal_shutdown("Shutting down everything!")
      #sys.exit()
      
if __name__ == '__main__':
  d = EdgeClient()
  d.register_exit_function()
  print("starting maint hread")
  #d.start_receive_thread()
  d.start_main_thread()
  time.sleep(5)
  d.end_threads()

  
