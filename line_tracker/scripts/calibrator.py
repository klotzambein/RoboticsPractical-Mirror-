#!/usr/bin/env python3

## Originally written by Anton Mulder
## Modified by Rik Timmers and Marc Groefsema
import rospy
import cv2
import yaml
from tkinter import Tk, Button, Scale, HORIZONTAL, Frame, Label, Text, END
from sensor_msgs.msg import Image
from image_converter import ImageConverter
import numpy as np
import os
from std_srvs.srv import Empty, SetBool
import rospkg 
from utils import preprocess_image

class Calibrator(object):
    
    def __init__(self):
        self.rospack = rospkg.RosPack()
        self.image_converter = ImageConverter()

        self.gui = Tk()
        self.gui.title("Calibrator")
        
        self.window = Frame(self.gui, width = 400, height = 500).pack()
        
        slider_length = 200
        
        self.h_lower_scale = Scale(self.gui, length = slider_length, from_ = 0, to_ = 180, orient = HORIZONTAL)
        self.h_lower_scale.place(x = 100, y = 10)
        self.h_lower_label = Label(self.gui, text = "H lower value:").place(x = 10, y = 30)
        
        self.h_upper_scale = Scale(self.gui, length = slider_length, from_ = 0, to_ = 180, orient = HORIZONTAL)
        self.h_upper_scale.place(x = 100, y = 50)
        self.h_upper_scale.set(180)
        self.h_uppper_label = Label(self.gui, text = "H upper value:").place(x = 10, y = 70)
        
        self.s_lower_scale = Scale(self.gui, length = slider_length, from_ = 0, to_ = 255, orient = HORIZONTAL)
        self.s_lower_scale.place(x = 100, y = 130)
        self.s_lower_label = Label(self.gui, text = "S lower value:").place(x = 10, y = 150)
        
        self.s_upper_scale = Scale(self.gui, length = slider_length, from_ = 0 , to_ = 255, orient = HORIZONTAL)
        self.s_upper_scale.place(x = 100, y = 170)
        self.s_upper_scale.set(255)
        self.s_upper_label = Label(self.gui, text = "S upper value:").place(x = 10, y = 190)
        
        self.v_lower_scale = Scale(self.gui, length = slider_length, from_ = 0, to_ = 255, orient = HORIZONTAL)
        self.v_lower_scale.place(x = 100, y = 250)
        self.v_lower_label = Label(self.gui, text = "V lower value:").place(x = 10, y = 270)
        
        self.v_upper_scale = Scale(self.gui, length = slider_length, from_ = 0, to_ = 255, orient = HORIZONTAL)
        self.v_upper_scale.place(x = 100, y = 290)
        self.v_upper_scale.set(255)
        self.v_upper_label = Label(self.gui, text = "V upper value").place(x = 10, y = 310)
               
        self.close_button = Button(self.gui, text = "Quit", command = self.gui.quit)
        self.close_button.place(x = 180, y = 450)
                
        self.update_loop()
        self.gui.mainloop()
        
        
    def update_loop(self):
        image_msg = rospy.wait_for_message("/camera/image", Image) ## The Jetson can not show images, so this should always run on the PC
        image = self.image_converter.convert_to_opencv(image_msg)  
        #image = cv2.imread("tweety.png")   ## If you want to play with this calibrator tool, without using images from your robot, you can uncomment this line, and comment the 2 lines above. Then you can segment Tweety :)
        image = self.filter_image(image)
        cv2.imshow("Image", image)
        cv2.waitKey(1)
        self.gui.after(10, self.update_loop)
        
    def filter_image(self, image):
        image = preprocess_image(image)
        
        hsv_image  = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_hsv = np.asarray([self.h_lower_scale.get(), self.s_lower_scale.get(), self.v_lower_scale.get()])
        upper_hsv = np.asarray([self.h_upper_scale.get(), self.s_upper_scale.get(), self.v_upper_scale.get()])
        
        mask = cv2.inRange(hsv_image, lower_hsv, upper_hsv)
        filtered_image = cv2.bitwise_and(hsv_image, hsv_image, mask = mask)
        filtered_rgb_image = cv2.cvtColor(filtered_image, cv2.COLOR_HSV2BGR)
        return filtered_rgb_image
    

rospy.init_node("calibrator")
calibrator = Calibrator()
