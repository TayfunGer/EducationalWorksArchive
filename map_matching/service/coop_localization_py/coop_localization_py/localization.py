from math import sqrt
import sys

from numpy.core.defchararray import asarray
from numpy.lib.utils import info
from skimage.feature.util import DescriptorExtractor
from skimage.util.dtype import img_as_float
from matplotlib import pyplot as plt
import rclpy
from rclpy.node import Node 
import numpy as np
from skimage import data
from skimage import img_as_ubyte
from skimage.feature import (match_descriptors, corner_harris, corner_peaks, ORB, plot_matches)
from skimage.transform import estimate_transform, warp, EuclideanTransform
from skimage.measure import ransac
from nav_msgs.msg import OccupancyGrid
from numpy.lib.stride_tricks import as_strided
import geometry_msgs.msg
import tf2_msgs.msg
from tf2_ros import TransformBroadcaster

class Localization(Node):
    

    def __init__(self):
        # Initialize the node
        super().__init__('localization')
        
        # Subscribe to the `/robot1/map` and `/robot2/global_costmap/costmap` topics
        self.own_map_sub = self.create_subscription(OccupancyGrid, "/robot1/map", self.own_map_callback, 10)
        self.other_map_sub = self.create_subscription(OccupancyGrid, "/robot2/global_costmap/costmap", self.other_map_callback, 10)
        
        # Create a publisher for the `/robot1/tf_static` topic
        self.pub_tf = self.create_publisher(tf2_msgs.msg.TFMessage, "/robot1/tf_static", 10)
        
        # Create a TransformBroadcaster object
        self.tf_br =  TransformBroadcaster(self)
        
        # Initialize the numpy arrays
        self.own_img = np.ndarray
        self.other_img = np.ndarray
        self.own_keypoints = np.ndarray
        self.other_keypoints = np.ndarray
        self.own_descriptors = np.ndarray
        self.other_descriptors = np.ndarray
        self.matches = np.ndarray
        self.map_cords = geometry_msgs.msg.TransformStamped()
        self.good_matches = np.ndarray

    
    def own_map_callback(self, msg:OccupancyGrid):
        # extract keypoints and find matches to get the transformation
        self.own_img = self.occupancy_to_np(msg)
        plt.figimage(self.own_img)
        plt.imsave("py_map_rob1.png", self.own_img, cmap="Greys_r")
        self.own_keypoints, self.own_descriptors = self.extract_keypoints(self.own_img, self.own_keypoints, self.own_descriptors)
        self.other_keypoints, self.other_descriptors = self.extract_keypoints(self.other_img, self.own_keypoints, self.own_descriptors)
        self.find_matches()
        self.get_Transformation()
    
    def other_map_callback(self, msg:OccupancyGrid):
        # get the map from the other robot
        self.other_img = self.occupancy_to_np(msg)
        plt.figimage(self.other_img)
        plt.imsave("py_map_rob2.png", self.other_img, cmap="Greys_r")
    
    def occupancy_to_np(self, map:OccupancyGrid):
        # Convert the occupancy grid message to a numpy array
        heigth = map.info.height
        width = map.info.width
        own_img = np.ndarray(shape=(heigth,width), dtype=np.uint8)
        for row in range(heigth):
            for col in range(width):
                cell = map.data[row * width + col]
                if cell == 0:
                   own_img[row][col] = 255
                elif cell == 1:
                    own_img[row][col] = 0
                elif cell == 100:
                    own_img[row][col] = 0
                elif cell == -1:
                    own_img[row][col] = 255
                else:
                    own_img[row][col] = 255
        return own_img

    def extract_keypoints(self, img, keypoints, descriptor):
        # extract keypoints using ORB
        descriptor_extractor = ORB(n_keypoints= 1000)
        descriptor_extractor.detect_and_extract(img)
        keypoints = descriptor_extractor.keypoints
        descriptor = descriptor_extractor.keypoints
        return keypoints, descriptor
        

    def find_matches(self):
        # find keypoint matches
        self.matches = match_descriptors(self.own_descriptors, self.other_descriptors, cross_check=True)

    def get_Transformation(self):
        # calculate EuclideanTransform with keypoint matches 
        idx = self.extract_good_matches()
        
        for i in idx:
            self.good_matches = self.matches[i]
        print(self.good_matches)

        src = self.own_keypoints[self.matches[:, 0]][:, ::-1]
        dst = self.other_keypoints[self.matches[:, 1]][:, ::-1]
        model = EuclideanTransform()
        sucsess = model.estimate(src, dst)
        trans , inliners = ransac((src, dst), EuclideanTransform, min_samples=100, residual_threshold=1.5, max_trials=500)
        inliner_keypoints_own = self.own_keypoints[self.matches[inliners, 0]]
        inliner_keypoints_other = self.other_keypoints[self.matches[inliners, 1]]

        print(trans.translation)

        self.transmatrix_to_ros(trans)

        fig, ax = plt.subplots(2, 1)
        plt.gray()
        plot_matches(ax[0], self.own_img, self.other_img, self.own_keypoints, self.other_keypoints, self.matches[inliners], only_matches=True)

        ax[0].axis("off")
        ax[0].set_title("Good Matches")
        plt.show()

    def transmatrix_to_ros(self, trans:EuclideanTransform):
        # Convert transfomationmatrix into geometry_msgs.msg.TransformStamped()
        self.map_cords = geometry_msgs.msg.TransformStamped()
        self.map_cords.header.frame_id = "map"
        self.map_cords.child_frame_id = "robot2_map"
        self.map_cords.header.stamp = self.get_clock().now().to_msg()
        self.map_cords.transform.translation.x = trans.translation[0]
        self.map_cords.transform.translation.y = trans.translation[1]
        self.map_cords.transform.rotation.x = trans.rotation
        self.other_to_own_map()

    def other_to_own_map(self):
        map_cords_other = tf2_msgs.msg.TFMessage()
        map_cords_other.transforms.append(self.map_cords)
        self.pub_tf.publish(map_cords_other)
        self.tf_br.sendTransform(self.map_cords)
        map_cords_other.transforms.clear()

    def compute_distance(self, point1, point2):
        return sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


    def extract_good_matches(self):
        distance_pair_found = 0
        max_distance_pair = 0
        delta = 2
        matchidx = []
        Output = []
        for i in range(len(self.matches)):
            matchidx.clear()
            matchidx.append(i)
            distance_pair_found = 0
            point_own_1 = self.own_keypoints[self.matches[i,0]]
            point_other_1 = self.other_keypoints[self.matches[i,1]]
            distance1 = self.compute_distance(point_other_1, point_own_1)
            for j in range(len(self.matches)):
                point_own_2 = self.own_keypoints[self.matches[j,0]]
                point_other_2 = self.other_keypoints[self.matches[j,1]]
                distance2 = self.compute_distance(point_other_2,point_own_2)
                if(distance1 <= distance2 + delta and distance1 >= distance2 - delta):
                    distance_pair_found += 1
                    matchidx.append(j)
            if(max_distance_pair < distance_pair_found):
                max_distance_pair = distance_pair_found
                Output = matchidx
        return Output
                
def main(args=None):
    rclpy.init(args=args)

    localization = Localization() 

    rclpy.spin(localization)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    localization.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()