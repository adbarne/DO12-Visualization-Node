import rclpy
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import String
import time
import subprocess, shlex, psutil
from deep_orange_msgs.msg import PtReport
import os

t1 = time.time()

# INPUT DATA TITLES
data_desc = ['Time',
             'ENG RPM',
             'ENG oil pres',
             'ENG coolant temp',
             'Trans oil pres',
             'Trans oil temp',
             'Fuel pres']

# INPUT DATAFIELD VALUES
data_value = ["null",
              'null',
              "null",
              "null",
              'null',
              'null',
              'null']

# FREQUENCY OF DATAVIS
print_interval = 0.1

# init rosbag recorder
rosbag_topics = ['/deep_orange_messages/pt_report']
topics_str = ' '.join(rosbag_topics)
date = str(time.ctime(time.time()))
date = date.replace(' ','_')
date = date.replace(':','')
out_dir = ' --output ~/Desktop/PT_vis_rosbag2/PT_' + date + '/'
command = 'ros2 bag record ' + topics_str + out_dir 
os.system(command + '&')

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            PtReport, '/deep_orange_msgs/pt_report', 
            self.PtReport,
            10
        ) 

    def PtReport(self, msg):   
        global data_value        
        # CHANGE DATA VALUES HERE  
        data_value[0] = time.ctime(time.time())
        data_value[1] = msg.engine_rpm
        data_value[2] = msg.engine_oil_pressure
        data_value[3] = msg.engine_coolant_temperature
        data_value[4] = msg.transmission_oil_pressure
        data_value[5] = msg.transmission_oil_temperature
        data_value[6] = msg.fuel_pressure
        # calculate time between messages
        global t1
        delta_t = time.time() - t1
        if delta_t > print_interval: 
            for i in range(len(data_value)):                    
                print('%20s: %20s' % (data_desc[i], data_value[i]))
            t1 = time.time()

def main(args=None):
    
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
