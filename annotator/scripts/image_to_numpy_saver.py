import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from image_converter import ImageConverter
import sys
import os

class DataSaver(object):

    def __init__(self, file_name):
        self.image_converter = ImageConverter()
        self.data = []
        self.file_name = file_name
        rospy.on_shutdown(self.save)
        self.image = None
        self.image_subscriber = rospy.Subscriber("/camera/image", Image, self.image_callback, queue_size = 1)
        self.cmd_vel_subscriber = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size = 1)

    def image_callback(self, image_msg):
        self.image = self.image_converter.convert_to_opencv(image_msg)
       
     
    def cmd_vel_callback(self, cmd_vel_msg):
        if cmd_vel_msg.linear.x !=0 or cmd_vel_msg.angular.z != 0:
          self.data.append(self.image)
          print("Nr of images: {0}".format(len(self.data)))

    def save(self): 
        self.data = np.asarray(self.data)
        file_path = os.environ["HOME"] + "/data/{}.npy".format(self.file_name)
        np.save(file_path, self.data)
        print("Numpy array saved to {0}".format(file_path))

if __name__ == "__main__":
    rospy.init_node("image_saver")

    if len(sys.argv) != 2: 
        print("Add file name without .npy extension")
        exit()

    file_name = sys.argv[1]
    data_saver = DataSaver(file_name)
    rospy.spin()
