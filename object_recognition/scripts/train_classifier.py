import tensorflow as tf 
from tensorflow.keras import layers, models
import numpy as np 
import cv2
import os

# Set the number of outputs/classes to train with
NR_OF_CLASSES = 3

# Set the input size of the image
IMAGE_SIZE = 32

# You can change this to stop earlier or train for longer
NR_OF_EPOCHS = 15 

# You can change the batch size of training
BATCH_SIZE = 20

train_x_data = np.load(os.environ["HOME"] + "/data/train_classifier.npy")
train_y_data = np.load(os.environ["HOME"] + "/data/label_classifier.npy")


# Create a model 
model = models.Sequential()

# Add CNN layer(s) to the model

# Possibly add dropout, and/or max pooling

# Add a flatten layer

# Add Dense layer(s) to the model

# The final layer needs to be a softmax
model.add(layers.Dense(NR_OF_CLASSES, activation="softmax"))

model.compile(optimizer="adam", 
              loss="categorical_crossentropy", 
              metrics=["accuracy"])
model.fit(train_x_data, train_y_data, epochs=NR_OF_EPOCHS, batch_size = BATCH_SIZE, shuffle=True, verbose = 1)

# Save the model
save_path = os.path.join(os.environ["HOME"], "network_model")

if not os.path.exists(save_path):
  os.mkdir(save_path)

# This will save, and overwrite, the network model
model.save(os.path.join(save_path, "model_classifier.h5"))
model = None
print("done")

