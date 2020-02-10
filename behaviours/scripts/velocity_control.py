#!/usr/bin/env python3

import serial
import struct
import rospy
from dynamic_reconfigure.server import Server
from teensy.cfg import senderConfig
from my_msgs.msg import Vel
from geometry_msgs.msg import Twist
from timeloop import Timeloop
from datetime import timedelta

publisher = rospy.Publisher('PID_vel', Vel, queue_size=1)
wheel_radius = 0.03 #meters
track = 0.108  #distance between the wheels

#### **************  Put your tuned PID values here ************************************** 
cur_config = {}
cur_config['k_P'] = 0   #put your tuned PID values here
cur_config['k_I'] = 0   #put your tuned PID values here
cur_config['k_D'] = 0   #put your tuned PID values here
cur_config['translational'] = 0
cur_config['rotational'] = 0



#### **************  You only have to the following function ************************************** 
# return the left and right wheel velocities in rads/s based on the provided
# body velocities. Where translational is the forward speed in m/s and
# rotational the rotational speed around the z axis in rads/s.
def determine_wheel_command(translational, rotational):
    #left_wheel_vel = ...
    #right_wheel_vel = ...

    return left_wheel_vel, right_wheel_vel


### **************** All code below this line is already complete ******************************

#Update cur_config when new one is received and send it to the teensy
def config_callback(config, level):
    print(config)

    cur_config['k_P'] = config['k_P']
    cur_config['k_I'] = config['k_I']
    cur_config['k_D'] = config['k_D']
    cur_config['translational'] = config['translational']
    cur_config['rotational'] = config['rotational']

    vel_left, vel_right = determine_wheel_command(config['translational'], config['rotational'])
    send(vel_left, vel_right)
    return config
    
# Generate a Vel message containing the PID configuration and the target wheel velocities
def send(vel_left, vel_right):
    msg = Vel()
    msg.kP = cur_config['k_P']
    msg.kI = cur_config['k_I']
    msg.kD = cur_config['k_D']
    msg.left_vel = vel_left
    msg.right_vel = vel_right
    publisher.publish(msg)

#Update velocities when new ones are received and send them to the teensy
def callback(data):
    vel_left, vel_right = determine_wheel_command(data.linear.x, data.angular.z)
    cur_config['translational'] = data.linear.x
    cur_config['rotational'] = data.angular.z
    send(vel_left, vel_right)
    

tl = Timeloop()

#Resend the last configuration every 0.45s, if this does not happen the teensy will stop driving
# out of self preservation.
@tl.job(interval=timedelta(seconds=0.45))
def timed_callback():
    vel_left, vel_right = determine_wheel_command(cur_config['translational'], cur_config['rotational'])
    send(vel_left, vel_right)
   
rospy.init_node('Teensy_communicator', anonymous=True)

rospy.Subscriber("cmd_vel", Twist, callback)
srv = Server(senderConfig, config_callback)

#If this program is stopped it will stop the teensy from driving
import atexit
@atexit.register
def stop():
    msg = Twist()
    callback(msg)

tl.start(block = True)

rospy.spin()

