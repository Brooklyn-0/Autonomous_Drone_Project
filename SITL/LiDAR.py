import time
import subprocess

from datetime import datetime

class livox_python:
    def __init__(self, dir = '/home/odin/Test/Senior-Design/ws_livox', cp1 = None, cp2 = None, cp3 = None, cp4 = None):
         self.dir = dir
         #self.flight_dir = "/home/odin/Test/Senior-Design/Hydra/Flight_Logs"
         self.flight_dir = '/home/hackfest08/Documents/Senior-Design2/Hydra/Flight_Logs'
         self.cp1 = cp1
         self.cp2 = cp2
         self.cp3 = cp3
         self.cp4 = cp4;
    def source(self):
        self.cp4 = subprocess.Popen(["source", "./devel/setup.bash"]);
    def connect_lidar(self):
        self.cp1 = subprocess.Popen(["roslaunch", "--wait", "livox_ros_driver", "livox_lidar.launch"], cwd=self.dir)
    def start_rviz(self):
        self.cp2 = subprocess.Popen(["roslaunch", "livox_mapping", "mapping_mid.launch"], cwd=self.dir)
    def start_rviz_and_connect(self):
        self.start_rviz()
        self.connect_lidar()
    def start_record(self):
        dt = datetime.now().strftime("%m_%d_%Y_%H_%M_%S")
        self.cp3 = subprocess.Popen(["rosbag", "record", "-O", self.flight_dir+"/client_"+ dt + ".bag", "/livox/lidar"], cwd="./")
    def end_record(self):
        try:
            self.cp3.terminate()
        except(AttributeError):
            print("Attribute error")
            pass
    def disconnect_lidar(self):
        self.cp1.terminate()
    def terminate_all(self):
        try:
            self.disconnect_lidar()
            self.cp2.terminate()
            self.cp3.terminate()
        except(AttributeError):
            pass
# test = livox_python()        
# test.start_rviz_and_connect()
# time.sleep(2)
# test.start_record()
# time.sleep(200)
# test.terminate_all()


# pwd = '/home/archlinuxisbetter/Documents/Senior-Design/ws_livox'
# child_process2 = subprocess.Popen(["roslaunch", "livox_mapping", "mapping_mid.launch"], cwd=pwd)
# child_process1 = subprocess.Popen(["roslaunch", "livox_ros_driver", "livox_lidar.launch"], cwd=pwd)

# x = input()
# if x == 'r':
#     child_process3 = subprocess.Popen(["rosbag", "record", "livox/lidar"], cwd=pwd)

# y = input()
# if y == 'r':
#     child_process3.terminate()
# child_process3.terminate()

# child_process1.terminate()
# child_process2.terminate()

if "__main__" == __name__:
    d = livox_python()
    d.start_record()
    time.sleep(5)
    d.end_record()
