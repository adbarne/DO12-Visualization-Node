import rclpy
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import String, UInt8, Header, Bool, Int32
import time
import subprocess, shlex, psutil
from raptor_dbw_msgs.msg import BrakeCmd, AcceleratorPedalCmd, SteeringCmd
from deep_orange_msgs.msg import CtReport,RcToCt, CtReport, MiscReport
from autoware_auto_msgs.msg import TrajectoryPoint, VehicleKinematicState
from geometry_msgs.msg import Transform
import os
t1 = time.time()

# kinematic state GNSS (x,y,heading)
# steering fault report
# /rc_to_ct_info msg.track_condition

# INPUT COLUMNS FOR DATAFRAME HERE
data_desc = ['Time',
            'Ct state',
            'Sys state',
            'Acc pedal cmd',
            'Brk pedal cmd',
            'Gear cmd',             
            'Steer cmd',
            'Track cond',
            'X',
            'Y',
            'Yaw rate',
            'Longitudinal vel',
            'Joy stop',
            'HB stop',
            'Heartbeat']

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
print_interval = 0.1

# init rosbag recorder
rosbag_topics = ['/raptor_dbw_interface/accelerator_pedal_cmd',
                '/raptor_dbw_interface/brake_cmd',
                '/raptor_dbw_interface/steering_cmd',
                '/raptor_dbw_interface/ct_report',
                '/raptor_dbw_interface/gear_cmd',
                '/raptor_dbw_interface/rc_to_ct',
                '/vehicle/vehicle_kinematic_state',
                '/vehicle/emergency_joystick',
                '/vehicle/emergency_heartbeat']
topics_str = ' '.join(rosbag_topics)
date = str(time.ctime(time.time()))
date = date.replace(' ','_')
date = date.replace(':','')
out_dir = ' --output ~/Desktop/AS_vis_rosbag2/AS_' + date + '/'
command = 'ros2 bag record ' + topics_str + out_dir 
os.system(command + '&')

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.ct_report = self.create_subscription(
            CtReport,
            '/raptor_dbw_interface/ct_report',
            self.CtReport,
            10)
        self.misc_report = self.create_subscription(
            UInt8,
            '/raptor_dbw_interface/misc_report',
            self.MiscReport,
            10)                            
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
        self.kinematic = self.create_subscription(
            VehicleKinematicState,
            '/vehicle/vehicle_kinematic_state',
            self.VehicleKinematicState,
            10)     
        self.emergencyjoy = self.create_subscription(
            Bool,
            '/vehicle/emergency_joy',
            self.EmergencyJoy,
            10)
        self.emergencyhb = self.create_subscription(
            Bool,
            '/vehicle/emergency_heartbeat',
            self.EmergencyHeartbeat,
            10)  
        self.heartbeat = self.create_subscription(
            Int32,
            '/diagnostics/heartbeat',
            self.Heartbeat,
            10)  
        # self.subscription # prevent unused variable warning

    def CtReport(self, msg):        
        global data_value 
        data_value[0] = time.ctime(time.time())
        data_value[1] = msg.ct_state
        
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

    def MiscReport(self, msg):        
        global data_value 
        # data_value[8] = time.ctime(time.time())
        data_value[2] = msg.sys_state
        
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
            
    def AcceleratorPedalCmd(self, msg):        
        global data_value 
        # data_value[0] = time.ctime(time.time())
        data_value[3] = msg.pedal_cmd

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
        data_value[4] = msg.pedal_cmd

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
        data_value[5] = msg.gear_cmd
        
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
        data_value[6] = msg.angle_cmd
        
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
        data_value[7] = msg.track_cond
        
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
        data_value[8] = msg.state.x
        data_value[9] = msg.state.y
        data_value[10] = msg.heading_rate_rps
        data_value[11] = msg.state.longitudinal_velocity_mps

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

    def EmergencyJoy(self, msg):        
        global data_value 
        # data_value[12] = time.ctime(time.time())
        data_value[12] = msg.emergency_joystick

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

    def EmergencyHeartbeat(self, msg):        
        global data_value 
        # data_value[12] = time.ctime(time.time())
        data_value[13] = msg.emergency_heartbeat

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

    def Heartbeat(self, msg):        
        global data_value 
        # data_value[12] = time.ctime(time.time())
        data_value[14] = msg.diagnostic_heartbeat_curr_value

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
