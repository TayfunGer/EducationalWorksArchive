import rclpy
import carla
from rclpy.node import Node
from rclpy.publisher import Publisher
import glob
from math import fabs
import os
import sys
import pprint
from numpy.core.overrides import verify_matching_signatures
import tf2_py
import csv
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import geometry_msgs.msg
import carla_common.transforms as trans
import visualization_msgs.msg
from geometry_msgs.msg import TransformStamped
from carla_msgs_extended.msg import TransformsStamped
import ros_compatibility as roscomp
import tensorflow as tf
from tensorflow import keras
from carla_msgs_extended.msg import BoolStamped
from std_msgs.msg import Header
from carla_msgs_extended.msg import ScenarioState, LaneChangeParameter

MODEL_DIR_LABEL = "/home/tayfun/dev_ws/master_arbeit/saved_models/model_a_ego_6_a_other_6"


class Spec_Lane_Change(Node):
    def __init__(self):
        super().__init__('spec_lane_change_node')

        # Load model
        self.model_lane_change_label: keras.models.Model = keras.models.load_model(
            MODEL_DIR_LABEL)

        # Publisher
        self.publisher_ = self.create_publisher(
            BoolStamped, 'lane_change_allowed', 10)

        # Subscriber for lane change parameter
        self.state_sub = self.create_subscription(
            LaneChangeParameter, "/lane_change_parameter", self.lane_change_callback, 10)

    # Callback for the subscriber
    def lane_change_callback(self, msg: LaneChangeParameter):
        sample = {
            "v_front": msg.v_front,
            "v_ego": msg.v_ego,
            "v_behind": msg.v_behind,
            "d_front": msg.d_front,
            "d_behind": msg.d_behind,
        }
        lane_change_allowed = BoolStamped()

        input_dict = {name: tf.convert_to_tensor(
            [value]) for name, value in sample.items()}

        # predict fron input parameters
        prediction, verdict = self.predict(
            self.model_lane_change_label, input_dict)

        # Publish data after prediction
        lane_change_allowed.verdict = verdict
        lane_change_allowed.header = msg.header
        self.publisher_.publish(lane_change_allowed)

    def predict(self, model, input_data):
        prediction = model.predict(input_data, verbose=0)
        if prediction[0] >= 0.558:
            verdict = True
        else:
            verdict = False
        return prediction, verdict


def main(args=None):
    rclpy.init(args=args)

    spec_lane_change = Spec_Lane_Change()

    rclpy.spin(spec_lane_change)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    spec_lane_change.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
