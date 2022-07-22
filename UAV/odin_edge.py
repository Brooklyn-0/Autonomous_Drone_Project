import argparse
import csv
from datetime import datetime
import pickle
import os
import rospy
import socket
import std_msgs.msg
import sys
import threading
import traceback
import _thread
import time

from sensor_msgs.msg import PointCloud2
from queue import Queue as Queue
from queue import Empty

DEBUG=1
TESTING=1
MY_STR = "ack"

def debug(line):
  if(DEBUG):
    print(line)

def log_to_file(data):
  f = open('log.txt', "wb")
  f.write(data)
  f.close()
    
bEND = b"*?!@"
    
# Parser for command line arguments
parser = argparse.ArgumentParser(description='Process arguments for odin edge server.')
parser.add_argument("ip", help="ip address of server")
parser.add_argument("--name", help="", default="EDGE01")
args = parser.parse_args()
edge_server_ip = args.ip

#Jetson Nano: 10.42.0.23
#Alex Laptop: 192.168.1.58
#defaultLocation = ("10.42.0.23", 80)
DATA_SIZE = 900394 #DATA_SIZE shifts between 180393 and 180394

class EdgeServer(threading.Thread):
  '''ground computer server instance'''
  def __init__(self, ip='localhost', port=1028):
    # self.folder_name = 'Flight_Logs'
    # self.client_file = None
    # self.csv_writer = None
    # self.init_file()
    
    # set up a TCP server
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    self.ip = ip
    self.port = port
    self.sock.bind(('', self.port)) #change first param tgo self.ip
    self.sock.listen(1)
    debug("Listening for clients")

    if(TESTING):
      self.create_publisher()

  # def init_file(self):
  #   path = os.path.dirname(os.getcwd())
  #   path = path + '/' + self.folder_name
  #   if not os.path.exists(path):
  #       os.mkdir(path)

  #   file_name = datetime.now().strftime("%m_%d_%Y_%H-%M-%S") + '_server_log.csv'
  #   self.client_file = open(path + '/' + file_name, "a+")
  #   self.csv_writer = csv.writer(self.client_file)
  #   self.csv_writer.writerow(['rtt'])
    #detection time, frame size
      
  def create_publisher(self):
     '''
      create_publisher utilizes ros to create a publisher for the frames that will
      be used in livox_detection.
      Note that this is _less_ efficient than sending the data directly to 
      livox_detection. This function is simply for testing purposes.
      The data will be published and the subscriber in livox_detection will pull the
      frames and perform object detection on it. 
      '''
     rospy.init_node('livox_server', anonymous=False)
     self.odin_frame = rospy.Publisher('/livox/odin_frame', PointCloud2, queue_size=1)
    
  def start(self):
    self.run()

  def recv_end(self, socket) -> 'byte stream':
    '''
    Taken from https://code.activestate.com/recipes/408859/
    '''
    total_data = []
    while True:
      data = socket.recv(500000)
      #print("len data: ", len(data))
      if not data:
        debug("No data")
        sys.exit()
      if bEND in data:
        #debug("bEND found in data")
        total_data.append(data[:data.find(bEND)])
        break
      total_data.append(data)
      #debug("len of adding total_data array: " + str(len(total_data)))
      if len(total_data) > 1:
        #check if the data was split between last two "data packets"
        last_pair = total_data[-2] + total_data[-1]
        if bEND in last_pair:
          #debug("popping")
          total_data[-2] = last_pair[:last_pair.find(bEND)]
          total_data.pop()
          break
    #debug("len of total_data array: " + str(len(total_data)))
    return b''.join(total_data)
        
    
  def run(self):
    new_sock, addr = self.sock.accept()
    debug("Established connection")

    while True:
      try:
        data = self.recv_end(new_sock)
        log_to_file(data)
        
        print("total len data: ", len(data))
        data = pickle.loads(data)
        self.odin_frame.publish(data)

        time.sleep(2)
        
        # Send the acknowledgement
        ack_data = pickle.dumps(MY_STR)
        new_sock.send(ack_data)

      except Exception as e:
        traceback.print_exc()
        print(e)
        sys.exit()
    
    # frames_received = 0
    # try:
    #   while True:
    #     data = b''
    #     rcv_data = new_sock.recv()
        
    #     data_size = 0
    #     while data_size < DATA_SIZE:
    #       packet = new_sock.recv(DATA_SIZE)
    #       if(not packet):
    #         print("Client side has closed")
    #         sys.exit()
    #       print("len packet: ", len(packet))
    #       data_size += len(packet)
    #       data += packet
    #     try:
    #       print("frame data len: ", len(data))
    #       data = pickle.loads(data)
    #       frames_received += 1
    #       self.odin_frame.publish(data)
    #       if(TESTING):
    #         ack_data = pickle.dumps(MY_STR)
    #         new_sock.send(ack_data)

    #       print(str(frames_received) + " frame received")
    #     except pickle.UnpicklingError:
    #       print("pickling error occured")
    #       pass
        
    # except Exception as e:
    #   traceback.print_exc()
    #   print("Some other exception occured")
    #   sys.exit()

if __name__=="__main__":
  d = EdgeServer(ip=edge_server_ip)
  d.start()

