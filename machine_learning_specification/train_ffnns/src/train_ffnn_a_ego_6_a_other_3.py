import typing as _typing
import numpy as np
import tensorflow as tf
from tensorflow import keras
import pandas as pd
import ml_spec_lib.lib as ml
from ml_spec_lib.lib import ML_SPEC as ml

import os

UP_DIR = os.path.realpath(os.path.join(os.path.dirname(__file__), '..'))
FILE_NAME = UP_DIR + '/data/lane_change_training_data.csv'

dataframe = pd.read_csv(FILE_NAME)

dataframe_1_label = dataframe.drop(
    columns=["label", "label_1", "label_2", "label_3"])


uniform_data = ml.data_to_uniform_distribution(dataframe_1_label)

validation_dataframe, test_dataframe, training_dataframe = ml.split_data_into_validation_training_test(
    uniform_data)

training_dataset = ml.dataframe_to_dataset(training_dataframe)
validation_dataset = ml.dataframe_to_dataset(validation_dataframe)

# Merge the dataset into a batch
training_dataset = training_dataset.batch(32)
validation_dataset = validation_dataset.batch(32)

# Define Inputlayer
v_front = keras.Input(shape=(1,), name="v_front")
v_ego = keras.Input(shape=(1,), name="v_ego")
v_behind = keras.Input(shape=(1,), name="v_behind")
d_front = keras.Input(shape=(1,), name="d_front")
d_behind = keras.Input(shape=(1,), name="d_behind")

all_inputs = [v_front, v_ego, v_behind, d_front, d_behind]

# Normalize the data and add it to the featurespace
v_front_encoded = ml.encode_numerical_feature(
    v_front, "v_front", training_dataset)
v_ego_encoded = ml.encode_numerical_feature(v_ego, "v_ego", training_dataset)
v_behind_encoded = ml.encode_numerical_feature(
    v_behind, "v_behind", training_dataset)
d_front_encoded = ml.encode_numerical_feature(
    d_front, "d_front", training_dataset)
d_behind_encoded = ml.encode_numerical_feature(
    d_behind, "d_behind", training_dataset)

all_features = tf.keras.layers.concatenate([
    v_front_encoded,
    v_ego_encoded,
    v_behind_encoded,
    d_front_encoded,
    d_behind_encoded,
])

# Define the hiddenlayer
x = tf.keras.layers.Dense(32, activation='relu')(all_features)
x = tf.keras.layers.Dropout(0.5)(x)

# Define the outputlayer
output = tf.keras.layers.Dense(1, activation="hard_sigmoid")(x)

# Train the neural network
model = tf.keras.Model(all_inputs, output)
model.compile("adam", "binary_crossentropy", metrics=[
    "accuracy", tf.keras.metrics.Precision()])
model.fit(training_dataset, epochs=50,
          validation_data=validation_dataset)

# Save created model
model.save(
    "/home/tayfun/dev_ws/master_arbeit/saved_models/model_a_ego_6_a_other_3")
