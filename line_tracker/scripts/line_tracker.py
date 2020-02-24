import rospy  ## The ROS library for python
import cv2    ## The OpenCV library for python
import numpy as np  ## Numpy 
from image_converter import ImageConverter  ## import a class that can transform ros image messages to opencv images, and back
from sensor_msgs.msg import Image
from utils import * ## Import the preprocess function as implemented in utils.py
from geometry_msgs.msg import Twist ## The message definition for sending target velocities for the robot. 
                                    ## See http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html
                                    ## Use linear.x as forward velocity
                                    ## Use angular.z as rotational velocity
                                    ## Set the reset to zero (the default values are already zero)

## Write your line tracker here:
class LineTracker:

    def __init__(self): ## the constructor

        self.image_converter = ImageConverter()
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

        ## for the last part at object construction, make a image subscribe.
        topic_name = "/camera/image" ## For when working on the Jetson
        #topic_name = "/camera/uncompressed" ## For debugging on the PC
        self.image_subscriber = rospy.Subscriber(topic_name, Image, self.update, queue_size = 1)


    def update(self, image_msg):
        ## - Transform a ROS image to a numpy / opencv image:
        img = self.image_converter.convert_to_opencv(image_msg)  

        ## - preprocess the image (see utils.py)
        prepocessed_image = preprocess_image(img)
        
        ## - use hsv filtering to obtain a binary image
        filtered_image = #.....  (Hint: Take a look at the implementation of the calibrator tool.)

        ## - Post process the binary image (see utils.py)
        postprocessed_image = postprocess_image(filtered_image)
        ## - determin the error for the PID controller
        ## - send a "cmd_vel" message, containing the robot target velocities

        pass

    def pid_update(self):
        ## Do your PID calculations here
        pass

if __name__ == "__main__":
    rospy.init_node("line_tracker_node")

    linetracker = LineTracker() ## add more constructor arguments if needed
    rospy.spin()  ## prevent the program from shutting down



