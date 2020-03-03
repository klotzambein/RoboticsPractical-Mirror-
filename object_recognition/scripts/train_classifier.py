import tensorflow as tf 
from tensorflow.keras import layers, models
import numpy as np 
import cv2
import os
import sys

import matplotlib.pyplot as plt

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

new_images = []
new_labels = []
for (image, label) in zip(train_x_data, train_y_data):
  if (np.sum(image) <= 5):
    continue

  hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
  hsv1 = hsv.copy()
  hsv[:,:,2] -= 5
  hsv1[:,:,2] += 5
  new_images.append(cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR))
  new_labels.append(label)
  new_images.append(image)
  new_labels.append(label)
  new_images.append(cv2.cvtColor(hsv1, cv2.COLOR_HSV2BGR))
  new_labels.append(label)

train_x_data = np.array(new_images)
train_y_data = np.array(new_labels)

print((train_x_data.shape, train_y_data.shape))

train_x_data = train_x_data.astype(np.float32) / 255.0

# plt.figure(figsize=(10,10))
# for i in range(25):
#     plt.subplot(5,5,i+1)
#     plt.xticks([])
#     plt.yticks([])
#     plt.grid(False)
#     plt.imshow(cv2.cvtColor(train_x_data[i], cv2.COLOR_BGR2RGB), cmap=plt.cm.binary)
#     # The CIFAR labels happen to be arrays, 
#     # which is why you need the extra index
#     plt.xlabel(train_y_data[i])
# plt.show()


# sys.exit()


# Create a model 
model = models.Sequential()

# Add CNN layer(s) to the model
model.add(layers.Conv2D(16, (3,3), activation="relu", input_shape=(32, 32, 3)))

# Possibly add dropout, and/or max pooling
model.add(layers.MaxPooling2D((2, 2)))
# model.add(layers.Dropout(0.1))
# Add a flatten layer
model.add(layers.Flatten())
# Add Dense layer(s) to the model
model.add(layers.Dense(8, activation="relu"))

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

