import cv2
import numpy as np
    
## This function receives the image as obtained from the camera topic.
## Do your image preprocessing here, such that both your line-tracker and the hsv-calibrator tool can use the same preprocessing function
## As preprocessing step, this function should implement a gaussian blur.    <----------------
def preprocess_image(image):
    return cv2.GaussianBlur(image, (11, 11), 10)  ## Return the preprocessed image here


## This function receives the hsv-filtered image
## Do your image postprocessing here, by first applying a few erosion steps to remove noise pixels, and then applying a few dilation steps. <------------
def postprocess_image(image):
    kernel1 = np.ones((5,5), np.uint8)
    kernel2 = np.ones((10,10), np.uint8)
    erosion = cv2.dilate(image, kernel1, iterations = 3)
    dilation = cv2.erode(erosion, kernel2, iterations = 4)
    return dilation  ## Return the preprocessed image here