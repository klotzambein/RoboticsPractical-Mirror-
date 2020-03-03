import numpy as np
import os
from tensorflow.keras.models import load_model

# Image input size
IMAGE_SIZE = 32

# Load the model
path = os.path.join(os.environ["HOME"], "network_model")
model_path = os.path.join(path, "model_classifier.h5")

model = load_model(model_path)

# List to hold all the images to classify (a batch of input data)
input_data = []

# Create some random example images
black_image = np.zeros((IMAGE_SIZE, IMAGE_SIZE, 3), dtype = np.uint8)
white_image = np.ones((IMAGE_SIZE, IMAGE_SIZE, 3), dtype = np.uint8)

# Put all the images inside a list (This will be your cropped images)
input_data.append(black_image)
input_data.append(white_image)

# Convert the list to a numpy array, and float32 values (the images are normally uint8)
input_data = np.asarray(input_data, dtype=np.float32)

# @TODO Normalize your data

# Predict all the images, returns an array of the predictions for each image
prediction = model.predict(input_data)

print("Prediction: ", prediction)
