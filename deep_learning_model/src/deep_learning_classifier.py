#!/usr/bin/env python
# coding: utf-8

#importing os module
import os

#importing numpy and pandas for computation and storage
import numpy as np
import pandas as pd
import tensorflow as tf
from tensorflow import keras
#keras for CNN
from keras.layers import Dense, Conv2D, Flatten, MaxPooling2D
from keras.models import Sequential
from keras.preprocessing.image import ImageDataGenerator
from keras_preprocessing import image

#importing modules for supervised learning algorithms
from sklearn.ensemble import ExtraTreesClassifier
from sklearn.ensemble import GradientBoostingClassifier
from sklearn.ensemble import RandomForestClassifier

#importing module for computing accuracy and splitting dataset
from sklearn.metrics import accuracy_score
from sklearn.model_selection import train_test_split

#importing ROS modules
import rospy
import rospkg
from std_msgs.msg import String
from deep_learning_model.srv import CNN, CNNResponse


dataset = os.path.join(rospkg.RosPack().get_path('deep_learning_model'), 'datasets')
pre_trained_model_weight_file = os.path.join(rospkg.RosPack().get_path('deep_learning_model'), 'trained_model', 'CNN4layers_categorical_crossentropy_smaller.ckpt')
epochs = 2
validation_split = 0.8
IMG_HEIGHT = 64
IMG_WIDTH = 64
test_dataset = os.path.join(rospkg.RosPack().get_path('deep_learning_model'), 'test_dataset')
test_images = []

for images in os.listdir(test_dataset):
    image_file_name = test_dataset + '/' + images
    test_images.append(image_file_name)

#Creating the model
def create_model():
    global IMG_HEIGHT, IMG_WIDTH
    model = Sequential([
        Conv2D(8, 3, padding='same', activation='relu', input_shape=(IMG_HEIGHT, IMG_WIDTH, 3)),
        MaxPooling2D(),
        Conv2D(16, 3, padding='same', activation='relu'),
        MaxPooling2D(),
        Conv2D(32, 3, padding='same', activation='relu'),
        MaxPooling2D(),
        Conv2D(64, 3, padding='same', activation='relu'),
        MaxPooling2D(),
        Flatten(),
        Dense(512, activation='relu'),
        Dense(3, activation='softmax')])

#Compiling Model using optimizer and loss functions
    model.compile(optimizer='adam',
                    loss='categorical_crossentropy',
                    metrics=['accuracy'])
    model.summary()
    return model


def train_model(model):
    global pre_trained_model_weight_file, epochs, validation_split, IMG_HEIGHT, IMG_WIDTH, dataset    
    #Defining class labels
    class_labels = np.array(['Crazing', 'Inclusion', 'No Defect'])

    #Setting up directory and validation split for the dataset
    data_dir = dataset
    val_split = validation_split
    dataset_image_generator = ImageDataGenerator(rescale=1. / 255, horizontal_flip=True, vertical_flip=True,
                                                 validation_split=val_split)

    #Accessing directories to get images
    data_Cr_dir = os.path.join(data_dir, 'Crazing')  # directory with our Cr defect pictures
    data_In_dir = os.path.join(data_dir, 'Inclusion')  # directory with our In defect pictures
    data_Pa_dir = os.path.join(data_dir, 'No Defect')  # directory with our No defect pictures
 
    #Setting up batch size and image parameters
    batch_size_train = 600
    batch_size_test = 400
    epochs = epochs

    #Generating training and test dataset
    train_data_gen = dataset_image_generator.flow_from_directory(batch_size=batch_size_train, directory=data_dir,
                                                                 subset="training", shuffle=True,
                                                                 target_size=(IMG_HEIGHT, IMG_WIDTH),
                                                                 class_mode='categorical')
    val_data_gen = dataset_image_generator.flow_from_directory(batch_size=batch_size_test, directory=data_dir,
                                                               target_size=(IMG_HEIGHT, IMG_WIDTH),
                                                               class_mode='categorical', subset="validation")

    #  ******Loading checkpoint path******
    # filepath = "F:/Sem 3/AME 547/AME-505/CNN4layers_categorical_crossentropy_smaller.ckpt"
    # model.save(filepath, overwrite=True, include_optimizer=True)

    #Generating history of the model and fitting dataset
    # history = model.fit(
    #     train_data_gen,
    #     steps_per_epoch=batch_size_train,
    #     epochs=epochs,
    #     validation_data=val_data_gen, validation_steps=batch_size_test)

    model.load_weights(pre_trained_model_weight_file)

    #Getting validation accuracy
    loss, acc = model.evaluate(val_data_gen)
    return acc, model

def classify_image(model, image_dir):
    test_image = image.load_img(image_dir, target_size = (64, 64))
    test_image = image.img_to_array(test_image)
    test_image = np.expand_dims(test_image, axis = 0)
    test_image /= 255.
    Ans = model.predict(test_image)
    final_ans = Ans[0]
    dict = {}
    dict[0] = 'Crazing'
    dict[1] = 'Inclusion'
    dict[2] = 'No_Defect'
    return dict[np.argmax(final_ans)]

# model = create_model()
# accuracy, model = train_model(model)
# answer = classify_image(model, test_images[5])
# print(answer)

def deep_learning_service(request):
    global model, test_images
    if request.train == True:
        model = create_model()
        accuracy,model = train_model(model)
        resp = CNNResponse()
        resp.status = "done"
        return resp
    elif request.classify  == True:
        model = create_model()
        accuracy,model = train_model(model)
        index = request.image_number.data
        label = classify_image(model, test_images[index])
        status = 'image_classified'
        resp = CNNResponse()
        resp.status = status
        resp.class_label = label
        return resp

def classify_using_cnn_server():
    rospy.init_node('neural_network_classifier')
    server = rospy.Service('/classify_image', CNN, deep_learning_service)
    rospy.spin()

if __name__ == "__main__":
    classify_using_cnn_server()
    pass