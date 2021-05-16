#!/usr/bin/env python3
import rospy
import rosbag
import time
import bosdyn.client
import rosnode
import subprocess
from pinatapy import PinataPy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import InteractiveMarkerUpdate


class BagRecorder():
    def __init__(self):
        types = {'CameraInfo': CameraInfo, 'Image': Image, 'TFMessage': TFMessage, 'InteractiveMarkerUpdate': InteractiveMarkerUpdate}
        rospy.init_node(f"rosbag_recorder", anonymous=False)
        config_path = rospy.get_param("~config")
        print(config_path)
        sdk = bosdyn.client.create_standard_sdk('understanding-spot')
        self.robot = sdk.create_robot('192.168.50.3')
        with open("/home/spot/config/config") as f:
            for line in f:
                line = line.split('/')
                user = line[0].strip()
                password = line[1].strip()
                pinata_pub = line[2].strip()
                pinata_secret = line[3].strip()
        self.pinata = PinataPy(pinata_pub, pinata_secret)
        self.robot.authenticate(user, password)
        self.state_client = self.robot.ensure_client('robot-state')
        inf = rosnode.get_node_info_description('/rviz')
        inf = inf.split()
        print(inf)
        date = time.strftime('%x')
        date = date.split('/')
        file_name_full = f'/home/spot/rosbags/lesson_one_full_{date[0]}_{date[1]}_{date[2]}_{time.time()}'
        self.command_full = ['rosbag', 'record', f'--output-name={file_name_full}', '/spot/depth/back/camera_info', '/spot/depth/back/image', '/spot/depth/frontleft/camera_info', '/spot/depth/frontleft/image', '/spot/depth/frontright/camera_info', '/spot/depth/frontright/image', '/spot/depth/left/camera_info', '/spot/depth/left/image', '/spot/depth/right/camera_info', '/spot/depth/right/image', '/tf', '/tf_static', '/twist_marker_server/update']
        # self.topics_str = ''
        # types_name = []
        # topic = False
        # for p in inf:
        #     p = p.strip()
        #     if topic:
        #         if p[0] == '/':
        #             self.command_full.append(p)
        #             self.topics_str += f'{p} ' 
        #         # if p[0] == '[':
        #         #     types_name.append(p.split('/')[-1][:-1])
        #     if p == "Subscriptions:":
        #         topic = True
        #     if p == "Services:":
        #         topic = False
        # print(self.command_full)
        # print(types)
        self.recording = False
        i = 0
        self.bag = None
        # for top in topics:
        #     rospy.Subscriber(top, types[f'{types_name[i]}'], self.callback)
        #     i += 1

    # def callback(self, data):
    #     #print(data._connection_header['topic'])
    #     if self.bag is not None:
    #         self.bag.write(data._connection_header['topic'], data)

    def record(self):
        rospy.loginfo('Starting recorging')
        date = time.strftime('%x')
        date = date.split('/')
        file_name = f'/home/spot/rosbags/lesson_one_{date[0]}_{date[1]}_{date[2]}_{time.time()}'
        self.command = ['rosbag', 'record', f'--output-name={file_name}', '/tf', '/tf_static']
        rosbag_proc = subprocess.Popen(self.command)
        rosbag_proc_full = subprocess.Popen(self.command_full)
        power_on = True
        while power_on:
            power_on = self.robot.is_powered_on()
            time.sleep(1)
            #power_on = False
        rosbag_proc.terminate()
        rosbag_proc_full.terminate()
        rospy.loginfo('Finished recording')
        time.sleep(2)
        res = self.pinata.pin_file_to_ipfs(f'{file_name}.bag'')
        rospy.loginfo(f"Published to IPFS with hash: {res['IpfsHash']}")
        
    def spin(self):
        while True:
            try:
                power = self.robot.is_powered_on()
                #power = True
                if power:
                    self.record()
            except KeyboardInterrupt:
                exit()

BagRecorder().spin()