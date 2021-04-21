import rclpy
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import String
import time
import subprocess, shlex, psutil
import pandas as pd
from deep_orange_msgs.msg import PtReport

t1 = time.time()

# INPUT COLUMNS FOR DATAFRAME HERE
columns = ['Time','Linear x', 'Pose x', 'Pose y']

# INPUT MESSAGES FOR EACH COLUMN HERE
msgdata = ['time.ctime(time.time())','msg.linear.x', 'msg._x', 'msg._y']

# INPUT DESIRED TOPICS 
topics = ['/turtle1/cmd_vel','/turtle1/pose']

# INPUT DESIRED MESSAGE TYPES
msgtypes = ['Twist','Pose']


# FREQUENCY OF DATAVIS
frequency = 5

# create empty dataframe structure from columns
df = pd.DataFrame(columns=columns)

# init rosbag recorder
# topics_str = ' '.join(topics)
# command = 'ros2 bag record' + topics_str
# command = shlex.split(command)
# rosbag_proc = subprocess.Popen(command)

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        for (topic,msgtype) in zip(topics,msgtypes):
            self.subscription = self.create_subscription(
                eval(msgtype),
                topic,
                self.listener_callback,
                10)
        # self.subscription # prevent unused variable warning

    def listener_callback(self, msg):        
        # NEXT STEPS
        # add ros time as first column 'Time' 
        # only update df if delta time > 0.1 s 
        # append previous row of df, overwrite it with new data

        temp = []
        # loop through columns and messages 
        for (column,value) in zip(columns,msgdata): 
            # if value != message attribute
            try:                            
                temp.append(str({column:eval(value)}))            
            except:
                pass
            # print(temp)           

        # rewrite the string into dict format... there must be an easier way to write this... right?
        temp = ','.join(temp)
        temp = temp.replace('{','')
        temp = temp.replace('}','')
        temp = '{'+temp+'}'

        # append new data
        global df
        df = df.append(eval(temp),ignore_index=True)
        
        # calculate time between messages
        global t1
        t2 = time.time() - t1

        interval = 1/frequency
        if t2 > interval: 
            print(df.iloc[-15:])
            t1 = time.time()
        # print(time.time())

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
