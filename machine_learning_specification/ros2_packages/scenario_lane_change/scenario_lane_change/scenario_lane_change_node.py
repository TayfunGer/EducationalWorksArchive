# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from operator import imod
from typing import no_type_check
import rclpy
import carla
from rclpy.node import Node
from rclpy.publisher import Publisher
import agent_lib as agent
import glob
import os
import sys
import numpy as np
import csv
from numpy.core.overrides import verify_matching_signatures


try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import time
import carla_common.transforms as trans
from carla_ros_bridge.lidar import Lidar
import agent_lib as agent
from agents.navigation.controller import VehiclePIDController
from random import randint, random, uniform
from rosgraph_msgs.msg import Clock
from carla_msgs_extended.msg import ScenarioState, LaneChangeParameter
from std_msgs.msg import Header


class ScenarioLaneChange(Node):

    def __init__(self):
        super().__init__('lane_change')

        self.actor_list = []
        self.actor_list_own = []
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(5.0)
        self.world: carla.World = self.client.get_world()
        self.world: carla.World = self.client.load_world('Town04')
        self.settings = self.world.get_settings()
        self.settings.fixed_delta_seconds = 0.0125
        self.world.apply_settings(self.settings)

        self.blueprint_library = self.world.get_blueprint_library()
        self.map: carla.Map = self.world.get_map()
        self.spawnpoints = self.map.get_spawn_points()
        # ROS2 Parameters
        self.declare_parameter('vstart_random', True)
        self.declare_parameter('vdrive_random', False)
        self.declare_parameter('check_errmsg', False)
        self.declare_parameter('vmin_start', 30.0)
        self.declare_parameter('vmax_start', 80.0)
        self.declare_parameter('vmin_drive', 32.0)
        self.declare_parameter('vmax_drive', 37.0)

        # ROS2 publisher
        self.pub_end = self.create_publisher(
            ScenarioState, "/scenario_state", 10)

        self.pub_param = self.create_publisher(
            LaneChangeParameter, "/lane_change_parameter", 10)

        # CARLA Spawnpoints Vehicle and Spectator
        self.Spawnpointego = carla.Transform(carla.Location(
            x=-319.3, y=30.1, z=0.598), carla.Rotation(pitch=0.0, yaw=0.0, roll=0.000000))

        self.Spawnpointother = carla.Transform(carla.Location(
            x=-339.3, y=26.5, z=0.598), carla.Rotation(pitch=0.0, yaw=0.0, roll=0.000000))

        self.Spawnpointother_2 = carla.Transform(carla.Location(
            x=-289.3, y=26.5, z=4.598), carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))

        self.Spawnpointsensor = carla.Transform(carla.Location(
            x=0.0, y=0, z=0.598), carla.Rotation(pitch=0.0, yaw=0.0, roll=0.000000))

        self.spec_bp = self.blueprint_library.find(
            'sensor.other.collision')

        self.spawnPointSpectator = carla.Transform(carla.Location(
            x=-319.3, y=30.1, z=90.598), carla.Rotation(pitch=-90.0, yaw=0.0, roll=0.000000))
        self.spectator = self.world.get_spectator()
        self.spectator.set_transform(self.spawnPointSpectator)
        self.vector = carla.Vector3D()
        self.vector.y = 1.0

        self.wps = self.map.generate_waypoints(5.0)

        # Callback function reacts to every tick from the simulator
        self.callback_id = self.world.on_tick(
            lambda world_snapshot: self.Tick())

        self.spec_location = carla.Location(
            x=10.0, y=-57.5, z=0.598)

        self.ego_spawned = False
        self.other_spawned = False
        self.other2_spawned = False
        self.spec_spawned = False

        # Initilaise CARLA Actors
        self.ego = carla.Actor
        self.other = carla.Actor
        self.other2 = carla.Actor
        self.spec_follow = carla.Actor

        self.ego_id = int
        self.other_id = int
        self.other2_id = int
        self.spec_id = int

        # Set default speed
        self.other_speed = 30.0
        self.other2_speed = 30.0
        self.ego_speed = 30.0

        self.actor_list = self.world.get_actors()
        # Initilase counters
        self.count = 0
        self.true_count = 0
        self.false_count = 0

        # Get start time for the scenario
        self.time_start = self.get_clock().now().to_msg()

    # Callback
    def Tick(self):
        self.actor_list = self.world.get_actors()
        state = ScenarioState()
        # Check if all vehicles are spawned
        if len(self.actor_list.filter("vehicle.*")) != 3:
            state.state = state.INITIALIZING
            self.pub_end.publish(state)
            if self.ego_spawned == False:
                self.ego = self.Spawn_ego()
                self.ego_id = self.ego.id
                # Check ROS2 Parameter
                if self.get_parameter(
                        'vstart_random').get_parameter_value().bool_value:
                    self.ego_speed = uniform(self.get_parameter('vmin_start').get_parameter_value(
                    ).double_value, self.get_parameter('vmax_start').get_parameter_value().double_value)
                    # Turn off the phisics for startup
                    self.ego.set_target_velocity(
                        carla.Vector3D(self.ego_speed/3.6, 0, 0))
            if self.other2_spawned == False:
                self.other2 = self.Spawn_other_2()
                self.other2_id = self.other2.id
                # Check ROS2 Parameter
                if self.get_parameter(
                        'vstart_random').get_parameter_value().bool_value:
                    self.other2_speed = uniform(self.get_parameter('vmin_start').get_parameter_value(
                    ).double_value, self.get_parameter('vmax_start').get_parameter_value().double_value)
                    if self.ego_speed >= 105.0:
                        max_other2 = 120.0
                    else:
                        max_other2 = self.ego_speed + 30.0
                    self.other2_speed = uniform(
                        self.ego_speed - 30.0, max_other2)
                    # Turn off the phisics for startup
                    self.other2.set_target_velocity(
                        carla.Vector3D(self.other2_speed/3.6, 0.0, 0.0))
            if self.other_spawned == False:
                self.other = self.Spawn_other()
                self.other_id = self.other.id
                # Check ROS2 Parameter
                if self.get_parameter(
                        'vstart_random').get_parameter_value().bool_value:
                    self.other_speed = uniform(self.get_parameter('vmin_start').get_parameter_value(
                    ).double_value, self.get_parameter('vmax_start').get_parameter_value().double_value)
                    if self.ego_speed >= 93.0:
                        max_other = 120.0
                    else:
                        max_other = self.ego_speed + 30.0
                    self.other_speed = uniform(
                        self.ego_speed - 30.0, max_other)
                    # Turn off the phisics for startup
                    self.other.set_target_velocity(
                        carla.Vector3D(self.other_speed/3.6, 0.0, 0.0))
                    self.time_start = self.get_clock().now().to_msg()
            if self.spec_spawned == False:
                self.spec_follow = self.Spawn_spectator()
                self.spec_id = self.spec_follow.id
        else:
            # Check if all vehicles are spawned
            if (self.world.get_actor(self.ego_id) != None) or (self.world.get_actor(self.other_id) != None) or (self.world.get_actor(self.spec_id) != None):
                self.Drive()
                state.state = state.RUNNING
                self.pub_end.publish(state)
            # End of the scenario after 12 seconds Destroy all CARLA Actors and log the data
            if self.time_start.sec + 12 <= self.get_clock().now().to_msg().sec:
                state.state = state.COMPLETED
                self.pub_end.publish(state)
                self.Data_log()
                self.ego.destroy()
                self.other.destroy()
                self.spec_follow.destroy()
                self.other2.destroy()
                self.actor_list = []
                self.ego_spawned = False
                self.other_spawned = False
                self.spec_spawned = False
                self.other2_spawned = False
                time.sleep(2)
                self.ego = carla.Actor
                self.other = carla.Actor
                self.spec_follow = carla.Actor
                self.other2 = carla.Actor

    def Spawn_ego(self):
        self.ego_spawned = True
        ego = agent.Agent.spawn_vehicle(
            self.actor_list_own, self.blueprint_library, self.world, 'mkz_2020', 'ego', self.Spawnpointego)
        return ego

    def Spawn_other(self):
        self.other_spawned = True
        other = agent.Agent.spawn_vehicle(
            self.actor_list_own, self.blueprint_library, self.world, 'mkz_2020', 'other', self.Spawnpointother, '0,0,255')
        return other

    def Spawn_spectator(self):
        self.spec_spawned = True
        spec_follow: carla.Actor = self.world.spawn_actor(
            self.spec_bp, self.Spawnpointsensor, attach_to=self.ego, attachment_type=carla.AttachmentType.Rigid)
        return spec_follow

    def Spawn_other_2(self):
        self.other2_spawned = True
        other = agent.Agent.spawn_vehicle(
            self.actor_list_own, self.blueprint_library, self.world, 'mkz_2020', 'other2', self.Spawnpointother_2, '0,0,255')
        return other
    
    # Move the vehicles along the waypoints
    def Drive(self):
        self.ego.set_simulate_physics(True)
        if self.spec_spawned == True:
            self.spec_location = agent.Agent.spectator_follow_vehicle(
                self.spectator, self.spec_location, self.spec_follow, pitch=-90.0, yaw=0.0, range=250.0)
        next_wp_ego = agent.Agent.wp_ahead(self.ego, self.map)
        agent.Agent.follow_wp(vehicle_actor=self.ego,
                              next_waypoint=next_wp_ego, speed=self.ego_speed, ignore_junction=True, max_throttle=1., args_longitudinal={'K_P': 1.0, 'K_D': 1.0, 'K_I': 1.0}, args_lateral={
                                  'K_P': 1.0, 'K_D': 1.0, 'K_I': 1.0})
        next_wp_other = agent.Agent.wp_ahead(self.other, self.map)
        agent.Agent.follow_wp(vehicle_actor=self.other,
                              next_waypoint=next_wp_other, speed=self.other_speed, ignore_junction=True, max_throttle=1., args_lateral={
                                  'K_P': 1.0, 'K_D': 1.0, 'K_I': 1.0}, args_longitudinal={'K_P': 1.0, 'K_D': 1.0, 'K_I': 1.0})
        next_wp_other2 = agent.Agent.wp_ahead(self.other2, self.map)
        agent.Agent.follow_wp(vehicle_actor=self.other2,
                              next_waypoint=next_wp_other2, speed=self.other2_speed, ignore_junction=True, max_throttle=1.)

        wp_list_ego = agent.Agent.get_trajectory_planed_lane_change(
            self.ego, next_wp_ego, self.map, carla.Location(-248, 0, 0), 2.0, 20)
        agent.Agent.draw_traject(self.world, self.ego,
                                 wp_list=wp_list_ego, z=1.0)

    def Lane_change_left(self):
        self.spec_location = agent.Agent.spectator_follow_vehicle(
            self.spectator, self.spec_location, self.spec_follow, pitch=-90.0, yaw=0.0, range=70.0)
        next_wp_ego = agent.Agent.wp_lane_left(self.ego, self.map)
        agent.Agent.follow_wp(vehicle_actor=self.ego,
                              next_waypoint=next_wp_ego)
    
    # Calculate the braking distance and log the data
    def Data_log(self):
        params = LaneChangeParameter()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        self.count = self.count + 1
        speed_ego = self.ego.get_velocity().length()
        speed_other = self.other.get_velocity().length()
        speed_other2 = self.other2.get_velocity().length()
        distance_behind = self.other.get_location().distance(
            self.ego.get_location() + carla.Location(0, -3.6, 0))
        distance_front = self.other2.get_location().distance(
            self.ego.get_location() + carla.Location(0, -3.6, 0))

        params.d_front = distance_front
        params.d_behind = distance_behind
        params.v_ego = speed_ego
        params.v_behind = speed_other
        params.v_front = speed_other2
        self.pub_param.publish(params)
        # a_ego = 6.0 m/s² a_other = 6.0 m/s²
        breaking_distance_ego = self.calc_breaking_distance_ego(speed_ego, 6.0)
        breaking_distance_other = self.calc_breaking_distance_other(
            speed_other, 6.0, 1.0)
        spec_bool = 0
        # a_ego = 3.0 m/s² a_other = 3.0 m/s²
        breaking_distance_ego1 = self.calc_breaking_distance_ego(
            speed_ego, 3.0)
        breaking_distance_other1 = self.calc_breaking_distance_other(
            speed_other, 3.0, 1.0)
        spec_bool1 = 0
        # a_ego = 4.5 m/s² a_other = 4.5 m/s²
        breaking_distance_ego2 = self.calc_breaking_distance_ego(
            speed_ego, 4.5)
        breaking_distance_other2 = self.calc_breaking_distance_other(
            speed_other, 4.5, 1.0)
        spec_bool2 = 0
        # a_ego = 7.0 m/s² a_other = 5.0 m/s²
        breaking_distance_ego3 = self.calc_breaking_distance_ego(
            speed_ego, 7.0)
        breaking_distance_other3 = self.calc_breaking_distance_other(
            speed_other, 5.0, 1.0)
        spec_bool3 = 0
        # a_ego = 6.0 m/s² a_other = 3.0 m/s²
        breaking_distance_ego4 = self.calc_breaking_distance_ego(
            speed_ego, 6.0)
        breaking_distance_other4 = self.calc_breaking_distance_other(
            speed_other, 3.0, 1.0)
        spec_bool4 = 0
        if (distance_behind >= breaking_distance_other) and (distance_front >= breaking_distance_ego):
            spec_bool = 1

        if (distance_behind >= breaking_distance_other1) and (distance_front >= breaking_distance_ego1):
            spec_bool1 = 1

        if (distance_behind >= breaking_distance_other2) and (distance_front >= breaking_distance_ego2):
            spec_bool2 = 1

        if (distance_behind >= breaking_distance_other3) and (distance_front >= breaking_distance_ego3):
            spec_bool3 = 1

        if (distance_behind >= breaking_distance_other4) and (distance_front >= breaking_distance_ego4):
            spec_bool4 = 1

        fields = ['v_front', 'v_ego', 'v_behind',
                  'd_front', 'd_behind', 'label', 'label_1', 'label_2', 'label_3', 'label_4']
        params_dict = {'v_front': speed_other2, 'v_ego': speed_ego, 'v_behind': speed_other,
                       'd_front': distance_front, 'd_behind': distance_behind, 'label': spec_bool,
                       'label_1': spec_bool1, 'label_2': spec_bool2, 'label_3': spec_bool3, 'label_4': spec_bool4}
        print(params_dict)
        # Write the data to csv file
        with open('/home/tayfun/dev_ws/Data/lane_change_4.csv', 'a') as file:

            writer = csv.DictWriter(file, fieldnames=fields)
            writer.writerow(params_dict)
            file.close()

    def calc_breaking_distance_ego(self, v, a):
        breaking_distance = v**2 / (2*a)
        return breaking_distance

    def calc_breaking_distance_other(self, v, a, t):
        breaking_distance = v**2 / (2*a) + (t * v)
        return breaking_distance


def main(args=None):
    rclpy.init(args=args)

    scenario_lane_change = ScenarioLaneChange()

    rclpy.spin(scenario_lane_change)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    scenario_lane_change.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
