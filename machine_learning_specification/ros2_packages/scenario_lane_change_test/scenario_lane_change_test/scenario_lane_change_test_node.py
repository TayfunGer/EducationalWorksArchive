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

        self.tm = self.client.get_trafficmanager()
        self.blueprint_library = self.world.get_blueprint_library()
        self.map: carla.Map = self.world.get_map()
        self.spawnpoints = self.map.get_spawn_points()

        # ROS2 Parameters
        self.declare_parameter('vstart_random', True)
        self.declare_parameter('vdrive_random', False)
        self.declare_parameter('check_errmsg', False)
        self.declare_parameter('vmin_start', 30.0)  # 30
        self.declare_parameter('vmax_start', 80.0)  # 80
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
            x=-419.3, y=26.5, z=0.598), carla.Rotation(pitch=0.0, yaw=0.0, roll=0.000000))

        self.Spawnpointother_2 = carla.Transform(carla.Location(
            x=-219.3, y=26.5, z=4.598), carla.Rotation(pitch=0.0, yaw=0.0, roll=0.0))

        self.Spawnpointsensor = carla.Transform(carla.Location(
            x=0.0, y=0, z=0.598), carla.Rotation(pitch=0.0, yaw=0.0, roll=0.000000))

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

        # Initialise state spawned
        self.ego_spawned = False
        self.other_spawned = False
        self.other2_spawned = False
        self.spec_spawned = False

        # Initalise CARLA Actors
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

        # Initialise counter
        self.count = 0
        self.true_count = 0
        self.false_count = 0

        # Inialise states for lane_chage, lane_change_time, reaction_time and collision
        self.lane_change_state = 0
        self.data_logged = 0
        self.lane_change_time_saved = 0
        self.lane_change_time = 0
        self.reaction_time = 0
        self.reaction_time_saved = 0
        self.collision = 0

        # Get start time for the scenario
        self.time_start = self.get_clock().now().to_msg()

    # Callback
    def Tick(self):
        self.actor_list = self.world.get_actors()
        state = ScenarioState()
        if len(self.actor_list.filter("vehicle.*")) != 3:
            state.state = state.INITIALIZING
            self.data_logged = 0
            self.lane_change_state = 0
            self.lane_change__time_save = 0
            self.reaction_time_saved = 0
            self.collision = 0
            self.pub_end.publish(state)
            if self.ego_spawned == False:
                self.ego = self.Spawn_ego()
                self.ego_id = self.ego.id
                # Check the ROS2 parameter if it is true generate a uniformly
                # distributed number between 30-120
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
                # Check the ROS2 parameter if it is true generate a uniformly
                # distributed number between ego speed +- 30
                if self.get_parameter(
                        'vstart_random').get_parameter_value().bool_value:
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
                # Check the ROS2 parameter if it is true generate a uniformly
                # distributed number between ego speed +- 30
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
            if (self.world.get_actor(self.ego_id) != None) or (self.world.get_actor(self.other_id) != None) or (self.world.get_actor(self.spec_id) != None):
                self.Drive()
                state.state = state.RUNNING
                self.pub_end.publish(state)

            # End of the scenario after 20 seconds or other speed = 0.0
            # Destroy all CARLA Actors and log the data
            if self.other.get_velocity().length() == 0.0 or self.time_start.sec + 20 <= self.get_clock().now().to_msg().sec:
                state.state = state.COMPLETED
                self.pub_end.publish(state)
                self.log_collision()
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
        other2 = agent.Agent.spawn_vehicle(
            self.actor_list_own, self.blueprint_library, self.world, 'mkz_2020', 'other2', self.Spawnpointother_2, '0,0,255')
        return other2

    def Spawn_collision_sensor(self, actor: carla.Actor):
        collision_sensor = self.world.spawn_actor(self.collision_sensor_ego_bp,
                                                  carla.Transform(), attach_to=actor)
        collision_sensor.listen(
            lambda event: self.function_handler(event))
        return collision_sensor

    # Move the vehicles along the waypoints
    def Drive(self):
        if self.spec_spawned == True:
            self.spec_location = agent.Agent.spectator_follow_vehicle(
                self.spectator, self.spec_location, self.spec_follow, pitch=-90.0, yaw=0.0, range=250.0)
        next_wp_ego = agent.Agent.wp_ahead(self.ego, self.map)
        agent.Agent.follow_wp(vehicle_actor=self.ego,
                              next_waypoint=next_wp_ego, speed=self.ego_speed, ignore_junction=True, max_throttle=1., args_longitudinal={'K_P': 1.0, 'K_D': 1.0, 'K_I': 1.0}, args_lateral={
                                  'K_P': 1.0, 'K_D': 1.0, 'K_I': 1.0}, max_brake=1.0)
        next_wp_other = agent.Agent.wp_ahead(self.other, self.map)
        agent.Agent.follow_wp(vehicle_actor=self.other,
                              next_waypoint=next_wp_other, speed=self.other_speed, ignore_junction=True, max_throttle=1., args_lateral={
                                  'K_P': 1.0, 'K_D': 1.0, 'K_I': 1.0}, args_longitudinal={'K_P': 1.0, 'K_D': 1.0, 'K_I': 1.0}, max_brake=1.0)
        next_wp_other2 = agent.Agent.wp_ahead(self.other2, self.map)
        agent.Agent.follow_wp(vehicle_actor=self.other2,
                              next_waypoint=next_wp_other2, speed=self.other2_speed, ignore_junction=True, max_throttle=1., max_brake=1.0)

        if self.time_start.sec + 12 <= self.get_clock().now().to_msg().sec and self.map.get_waypoint(self.ego.get_location()).lane_id == 4:
            self.Lane_change_left()
            self.lane_change_state = 1
        if self.map.get_waypoint(self.ego.get_location()).lane_id == 3:
            if self.lane_change_time_saved == 0:
                self.lane_change_time = self.get_clock().now().to_msg().sec
                self.lane_change_time_saved = 1
            if self.lane_change_time + 2 <= self.get_clock().now().to_msg().sec:
                self.lane_change_state = 2
                if self.reaction_time_saved == 0:
                    self.reaction_time = self.get_clock().now().to_msg().sec
                    self.reaction_time_saved = 1
                self.ego_speed = 0.0
                self.other2_speed = 0.0
            if self.reaction_time_saved == 0 and self.lane_change_state == 2:
                self.reaction_time = self.get_clock().now().to_msg().sec
                self.reaction_time_saved = 1
            # Wait for the reaction time before starting the braking for the other_vehicle
            if self.reaction_time + 1 <= self.get_clock().now().to_msg().sec:
                self.other_speed = 0.0
            d_o_o2 = self.other.get_location().distance(self.other2.get_location())
            d_o_ego = self.other.get_location().distance(self.ego.get_location())
            d_ego_o2 = self.ego.get_location().distance(self.other2.get_location())
            if d_o_o2 < 5.0 or d_o_ego < 5.0 or d_ego_o2 < 5.0:
                self.collision = 1

    def Lane_change_left(self):
        self.spec_location = agent.Agent.spectator_follow_vehicle(
            self.spectator, self.spec_location, self.spec_follow, pitch=-90.0, yaw=0.0, range=250.0)
        next_wp_ego = agent.Agent.wp_lane_left(self.ego, self.map)
        agent.Agent.follow_wp(vehicle_actor=self.ego,
                              next_waypoint=next_wp_ego)

    def calc_breaking_distance_ego(self, v, a):
        breaking_distance = v**2 / (2*a)
        return breaking_distance

    def calc_breaking_distance_other(self, v, a, t):
        breaking_distance = v**2 / (2*a) + (t * v)
        return breaking_distance

    # log collisions in CSV filte
    def log_collision(self):
        fields = ["collision", "prediction"]
        collision_dict = {"collision": self.collision}
        with open('/home/tayfun/dev_ws/Data/collision_auswertung.csv', 'a') as file:
            writer = csv.DictWriter(file, fieldnames=fields)
            writer.writerow(collision_dict)
            file.close()


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
