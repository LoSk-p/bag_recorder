#!/usr/bin/env python3
import rospy
import rosbag
import time
import bosdyn.client
import rosnode
import subprocess
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import InteractiveMarkerUpdate


class BagRecorder():
    def __init__(self):
        types = {'CameraInfo': CameraInfo, 'Image': Image, 'TFMessage': TFMessage, 'InteractiveMarkerUpdate': InteractiveMarkerUpdate}
        rospy.init_node(f"rosbag_recorder", anonymous=False)
        sdk = bosdyn.client.create_standard_sdk('understanding-spot')
        self.robot = sdk.create_robot('192.168.50.3')
        config_path = rospy.get_param("~config")
        with open(config_path) as f:
            for line in f:
                line = line.split('/')
                user = line[0].strip()
                password = line[1].strip()
        self.robot.authenticate(user, password)
        self.state_client = self.robot.ensure_client('robot-state')
        inf = rosnode.get_node_info_description('/rviz')
        inf = inf.split()
        print(inf)
        date = time.strftime('%x')
        date = date.split('/')
        self.command = ['rosbag', 'record', f'--output-name=/home/spot/rosbags/lesson_one_{date[0]}_{date[1]}_{date[2]}_{time.time()}']
        self.topics_str = ''
        types_name = []
        topic = False
        for p in inf:
            p = p.strip()
            if topic:
                if p[0] == '/':
                    self.command.append(p)
                    self.topics_str += f'{p} ' 
                # if p[0] == '[':
                #     types_name.append(p.split('/')[-1][:-1])
            if p == "Subscriptions:":
                topic = True
            if p == "Services:":
                topic = False
        print(self.command)
        print(types)
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
        self.command = ['rosbag', 'record', f'--output-name=/home/spot/rosbags/lesson_one_{date[0]}_{date[1]}_{date[2]}_{time.time()}', '/tf', '/tf_static']
        rosbag_proc = subprocess.Popen(self.command)
        power_on = True
        while power_on:
            power_on = self.robot.is_powered_on()
            time.sleep(1)
            #power_on = False
        rosbag_proc.terminate()
        rospy.loginfo('Finished recording')
        
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