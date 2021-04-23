import rclpy
from rclpy.time import Time
from rclpy.node import Node
from std_msgs.msg import String
import time
import matplotlib.pyplot as plt
import subprocess, shlex, psutil
from raptor_dbw_msgs.msg import SteeringReport,Brake2Report, BrakeCmd
from autoware_auto_msgs.msg import VehicleKinematicState
import os

t1 = time.time()
yaw_rate = []
# INPUT COLUMNS FOR DATAFRAME HERE
data_desc = ['Time',      # always print time!
           'Brake cmd',
           'Front BRK pres',
           'Rear BRK pres',
           'Steer motor cmd',
           'Steer motor fdbk',
           'Long. vel',
           'Yaw rate']

           # Add azimuth rate from the GNSS, 
           # Vehicle speed from motec,
           #  Steering motor demand as well as feedback, 
           #  brake pressure demand as well as feedback, 
           # Lateral speed 
           # (we can give you the formual to calulate...
           # from Velocty north, Velocity east, Azimuth & Azimuth rate)

# INPUT MESSAGES FOR EACH COLUMN HERE
data_value = ['null',   # always print time!
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

# FREQUENCY OF DATAVIS PER EACH CALLBACK
print_interval = 0.1

# init rosbag recorder
rosbag_topics = ['/raptor_dbw_interface/brake_cmd',
                '/raptor_dbw_interface/brake_2_report',
                '/raptor_dbw_interface/steering_report',
                '/vehicle/vehicle_kinematic_state']

topics_str = ' '.join(rosbag_topics)
date = str(time.ctime(time.time()))
date = date.replace(' ','_')
date = date.replace(':','')
out_dir = ' --output ~/Desktop/VD_vis_rosbag2/VD_' + date + '/'
command = 'ros2 bag record ' + topics_str + out_dir 
os.system(command + ' &')

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            BrakeCmd,
            '/raptor_dbw_interface/brake_cmd',
            self.BrakeCmd,
            10)   
        self.brake_2_report = self.create_subscription(
            Brake2Report,
            '/raptor_dbw_interface/brake_2_report',
            self.Brake2Report,
            10)
        self.steering_report = self.create_subscription(
            SteeringReport,
            '/raptor_dbw_interface/steering_report',
            self.SteeringReport,
            10)     
        self.gnss = self.create_subscription(
            VehicleKinematicState,
            '/vehicle/vehicle_kinematic_state',
            self.VehicleKinematicState,
            10)                        
        # self.subscription # prevent unused variable warning
 
    def BrakeCmd(self, msg):   
        global data_value     
        data_value[0] = time.ctime(time.time())
        data_value[1] = msg.pedal_cmd

        def plot_data(x):

            # create sliding window for x
            x = x[-30:]
            
            # clear figure
            plt.clf()

            # plot data                        
            plt.plot(x)
            plt.title('Yaw rate')
            plt.axis('equal')
            plt.draw()
            plt.pause(0.001)
            
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

    def Brake2Report(self, msg):   
        global data_value     
        # data_value[2] = time.ctime(time.time())
        data_value[2] = msg.front_brake_pressure
        data_value[3] = msg.rear_brake_pressure

        def plot_data(x):

            # create sliding window for x
            x = x[-30:]
            
            # clear figure
            plt.clf()

            # plot data                        
            plt.plot(x)
            plt.title('Yaw rate')
            plt.axis('equal')
            plt.draw()
            plt.pause(0.001)
            
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

        # # append yaw rate data
        # yaw_rate.append()
        # plot_data(yaw_rate)
        
    def SteeringReport(self, msg):   
        global data_value
        data_value[4] = msg.steering_wheel_angle_cmd
        data_value[5] = msg.steering_wheel_angle

        def plot_data(x):

            # create sliding window for x
            x = x[-30:]
            
            # clear figure
            plt.clf()

            # plot data                        
            plt.plot(x)
            plt.title('Yaw rate')
            plt.xlabel('')
            plt.axis('equal')
            plt.draw()
            plt.pause(0.001)
            
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
        data_value[6] = msg.state.longitudinal_velocity
        data_value[7] = msg.heading_rate_rps
        
        def plot_data(x):

            # create sliding window for x
            x = x[-30:]
            
            # clear figure
            plt.clf()

            # plot data                        
            plt.plot(x)
            plt.title('Yaw rate')
            plt.xlabel('')
            plt.axis('equal')
            plt.draw()
            plt.pause(0.001)
            
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
        # append yaw rate data
        yaw_rate.append(msg.heading_rate_rps)
        plot_data(yaw_rate)
        
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
