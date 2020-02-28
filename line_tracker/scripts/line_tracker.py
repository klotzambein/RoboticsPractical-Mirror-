import os
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

publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)

class LineTracker:

    def __init__(self): ## the constructor

        self.image_converter = ImageConverter()
        ## initialize the PID parameters
        self.p = 0.1
        self.i = 0.005
        self.d = 0.0009
        self.prevError = 0
        self.integral = 0

        ## initialize the HSV thesholding parameters
        self.h_min = 15
        self.h_max = 40
        self.s_min = 85
        self.s_max = 200
        self.v_min = 111
        self.v_max = 255

        ## for the last part at object construction, make a image subscribe.
        if "nano-sudo" in os.uname()[1]:
            self.topic_name = "/camera/image" ## For when working on the Jetson
        else:
            self.topic_name = "/camera/uncompressed" ## For debugging on the PC
        self.image_subscriber = rospy.Subscriber(self.topic_name, Image, self.update, queue_size = 1)

    def filter_image(self, image):
        hsv_image  = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_hsv = np.asarray([self.h_min, self.s_min, self.v_min])
        upper_hsv = np.asarray([self.h_max, self.s_max, self.v_max])
        
        mask = cv2.inRange(hsv_image, lower_hsv, upper_hsv)
        return mask


    def update(self, image_msg):
        ## - Transform a ROS image to a numpy / opencv image:
        img = self.image_converter.convert_to_opencv(image_msg)  

        ## - preprocess the image (see utils.py)
        prepocessed_image = preprocess_image(img)
        
        ## - use hsv filtering to obtain a binary image
        filtered_image = self.filter_image(prepocessed_image)

        ## - Post process the binary image (see utils.py)
        postprocessed_image = postprocess_image(filtered_image)
        ## - determin the error for the PID controller
        ## - send a "cmd_vel" message, containing the robot target velocities

        normalize_img = postprocessed_image / 255
        count_column_img = np.count_nonzero(normalize_img, axis=0)
        sum_count_img = np.sum(count_column_img)

        if (sum_count_img < 300):
            rotation = 0
        else:
            multiplier = np.linspace(-1.0, 1.0, num=count_column_img.size)

            error = np.mean(count_column_img * multiplier)

            rotation = self.pid_update(error)
 

        msg = Twist()
        msg.linear.x = 0.14
        msg.angular.z = -rotation

        print(msg)

        publisher.publish(msg)


        pass

    def pid_update(self, error):
        p = error
        i = self.integral + error
        d = error - self.prevError
        self.prevError = error
        self.integral = i
        return self.p * p + self.i * i + self.d * d

if __name__ == "__main__":
    rospy.init_node("line_tracker_node")

    linetracker = LineTracker() ## add more constructor arguments if needed
    rospy.spin()  ## prevent the program from shutting down



