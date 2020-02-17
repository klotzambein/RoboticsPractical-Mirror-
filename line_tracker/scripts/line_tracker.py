import rospy  ## The ROS library for python
import cv2    ## The OpenCV library for python
import numpy as np  ## Numpy 
from utils import preprocess_image ## Import the preprocess function as implemented in utils.py
from geometry_msgs.msg import Twist ## The message definition for sending target velocities for the robot. 
                                    ## See http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html
                                    ## Use linear.x as forward velocity
                                    ## Use angular.z as rotational velocity
                                    ## Set the reset to zero (the default values are already zero)

## Write your line tracker here:
class LineTracker:

    def __init__(self): ## the constructor
        ## initialize the PID parameters
        #self.p = ..
        #self.i = ..
        #self.d = ..

        ## initialize the HSV thesholding parameters
        #self.h_min = ..
        #self.h_max = ..
        #self.s_min = ..
        #self.s_max = ..
        #self.v_min = ..
        #self.v_max = ..

    def update(self):
        ## - obtain a new image from the camera
        ## - preprocess the image
        ## - use hsv filtering
        ## - determin the error for the PID controller
        ## - send a "cmd_vel" message, containing the robot target velocities

        pass

    def pid_update(self):
        ## Do your PID calculations here
        pass

if __name__ == "__main__":
    rospy.init_node("line_tracker_node")

    linetracker = LineTracker() ## add more constructor arguments if needed
    rate = rospy.Rate(20)  ## This sets the update frequency to 20 Hz, you can adjust this if you want to.
    while not rospy.is_shutdown():  ## While the program shouldn't stop:
        linetracker.update()        ## update the linetracker
        rate.sleep()                ## wait until the next iteration should start, according to the earlier defined update frequency



