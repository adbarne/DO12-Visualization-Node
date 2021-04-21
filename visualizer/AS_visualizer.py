import rclpy
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import String, UInt8, Header
import time
import subprocess, shlex, psutil
from raptor_dbw_msgs.msg import BrakeCmd, AcceleratorPedalCmd, SteeringCmd
from deep_orange_msgs.msg import CtReport,RcToCt, CtReport
from autoware_auto_msgs.msg import TrajectoryPoint, VehicleKinematicState
from geometry_msgs.msg import Transform
from novatel_gps_msgs.msg import NovatelPosition

t1 = time.time()

# kinematic state GNSS (x,y,heading)
# steering fault report
# /rc_to_ct_info msg.track_condition

# INPUT COLUMNS FOR DATAFRAME HERE
data_desc = ['Time',
           'Acc pedal cmd',
           'Brk pedal cmd',
           'Gear cmd',             
           'Steer cmd',
           'CT state',
           'Track cond',
           'X',
           'Y',
           'Yaw rate',
           'GNSS sln type'
           '']

# INPUT MESSAGES FIELDS
data_value = ['null',
           'null',
           'null',
           'null',
           'null',
           'null',
           'null',
           'null',
           'null',
           'null',
           'null',
           'null',
           'null',
           'null',
           'null',
           'null',
           'null',
           'null',
           'null']

# FREQUENCY OF DATAVIS
print_interval = 0.2

# init rosbag recorder
# topics_str = ' '.join(topics)
# command = 'ros2 bag record' + topics_str
# command = shlex.split(command)
# rosbag_proc = subprocess.Popen(command)

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.accelerator_pedal_cmd = self.create_subscription(
            AcceleratorPedalCmd,
            '/raptor_dbw_interface/accelerator_pedal_cmd',
            self.AcceleratorPedalCmd,
            10)
        self.subscription = self.create_subscription(
            BrakeCmd,
            '/raptor_dbw_interface/brake_cmd',
            self.BrakeCmd,
            10)   
        self.steering_cmd = self.create_subscription(
            SteeringCmd,
            '/raptor_dbw_interface/steering_cmd',
            self.SteeringCmd,
            10)
        self.ct_report = self.create_subscription(
            CtReport,
            '/raptor_dbw_interface/ct_report',
            self.CtReport,
            10)
        self.gear_cmd = self.create_subscription(
            UInt8,
            '/raptor_dbw_interface/gear_cmd',
            self.GearCmd,
            10)  
        self.rc_to_ct = self.create_subscription(
            RcToCt,
            '/raptor_dbw_interface/rc_to_ct',
            self.RcToCt,
            10) 
        self.gnss = self.create_subscription(
            VehicleKinematicState,
            '/vehicle/vehicle_kinematic_state',
            self.VehicleKinematicState,
            10)    
        # self.gnss = self.create_subscription(
        #     NovatelPosition,
        #     '/bestpos',
        #     self.NovatelPosition,
        #     10)             
        # self.subscription # prevent unused variable warning

    def AcceleratorPedalCmd(self, msg):        
        global data_value 
        data_value[0] = time.ctime(time.time())
        data_value[1] = msg.pedal_cmd

        # calculate time between messages
        global t1
        dt = time.time() - t1
        if dt > print_interval: 
            for i in range(len(data_value)):
                try:
                    print('%20s: %20s' % (data_desc[i], data_value[i]))  
                except:
                    pass
            t1 = time.time()

    def BrakeCmd(self, msg):
        global data_value 
        # data_value[2] = time.ctime(time.time())
        data_value[2] = msg.pedal_cmd

        # calculate time between messages
        global t1
        dt = time.time() - t1
        if dt > print_interval: 
            for i in range(len(data_value)):
                try:
                    print('%20s: %20s' % (data_desc[i], data_value[i]))  
                except:
                    pass
            t1 = time.time()        

    def GearCmd(self, msg):        
        global data_value 
        # data_value[4] = time.ctime(time.time())
        data_value[3] = msg.gear_cmd
        
        # calculate time between messages
        global t1
        dt = time.time() - t1
        if dt > print_interval: 
            for i in range(len(data_value)):
                try:
                    print('%20s: %20s' % (data_desc[i], data_value[i]))  
                except:
                    pass
            t1 = time.time()

    def SteeringCmd(self, msg):        
        global data_value 
        # data_value[6] = time.ctime(time.time())
        data_value[4] = msg.angle_cmd
        
        # calculate time between messages
        global t1
        dt = time.time() - t1
        if dt > print_interval: 
            for i in range(len(data_value)):
                try:
                    print('%20s: %20s' % (data_desc[i], data_value[i]))  
                except:
                    pass
            t1 = time.time()

    def CtReport(self, msg):        
        global data_value 
        # data_value[8] = time.ctime(time.time())
        data_value[5] = msg.ct_state
        
        # calculate time between messages
        global t1
        dt = time.time() - t1
        if dt > print_interval: 
            for i in range(len(data_value)):
                try:
                    print('%20s: %20s' % (data_desc[i], data_value[i]))  
                except:
                    pass
            t1 = time.time() 

    def RcToCt(self, msg):        
        global data_value 
        # data_value[10] = time.ctime(time.time())
        data_value[6] = msg.track_cond
        
        # calculate time between messages
        global t1
        dt = time.time() - t1
        if dt > print_interval: 
            for i in range(len(data_value)):
                try:
                    print('%20s: %20s' % (data_desc[i], data_value[i]))  
                except:
                    pass
            t1 = time.time() 

    def VehicleKinematicState(self, msg):        
        global data_value 
        # data_value[12] = time.ctime(time.time())
        data_value[7] = msg.state.x
        data_value[8] = msg.state.y
        data_value[9] = msg.heading_rate_rps
        data_value[10] = msg.position_type

        # calculate time between messages
        global t1
        dt = time.time() - t1
        if dt > print_interval: 
            for i in range(len(data_value)):
                try:
                    print('%20s: %20s' % (data_desc[i], data_value[i]))  
                except:
                    pass
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
