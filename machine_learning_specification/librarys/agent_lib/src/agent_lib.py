from re import T
from typing import List, no_type_check
import rclpy
import carla
from rclpy.node import Node
from rclpy.publisher import Publisher

import glob
from math import cos, fabs, sin
import os
import sys
import numpy as np

from numpy.core.overrides import verify_matching_signatures
import tf2_py


try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import time
import tf2_ros
import geometry_msgs.msg
import tf2_msgs.msg
import carla_common.transforms as trans
import tf2_msgs
from carla_ros_bridge.lidar import Lidar
from agents.navigation.controller import VehiclePIDController
from agents.navigation.local_planner import LocalPlanner
from random import randint, random
from rosgraph_msgs.msg import Clock
from runtime_verification_msgs.msg import Verdicts


class Agent:
    def __init__(self):
        pass

    def follow_wp(vehicle_actor: carla.Actor, next_waypoint: carla.Waypoint, speed: float = 30.0, args_lateral={
            'K_P': 1.0, 'K_D': 1.0, 'K_I': 1.0}, args_longitudinal={'K_P': 1.0, 'K_D': 1.0, 'K_I': 1.0}, offset=0, max_throttle=0.75, max_brake=1.0, max_steering=0.75, ignore_junction=False):
        '''
        follows passed waypoint

        Params:
            vehicle_actor (carla.Actor): the vehicle which follows the waypoints
            next_waypoint (carla.Waypoint): the next waypoint where to go
            speed (int): speed of Vehicle for PID controller
            args_lateral ({
            'K_P': float, 'K_D': float, 'K_I': float}): lateral parameters for the PID controler
            args_longitudinal ({
            'K_P': float, 'K_D': float, 'K_I': float}): longitdinal parameters for the PID controler
            offset (int): offset for the PID controler
            max throttel (float: 0.0 to 1.0): Percentage value indicating the maximum load on the throttel
            max break (float: 0.0 to 1.0): percentage which indicates the maximum load on the brake
            max stearing (float: 0.0 to 1.0): percentage specification of the maximum steering angle
        '''
        custom_controller = VehiclePIDController(
            vehicle_actor, args_lateral, args_longitudinal, offset, max_throttle, max_brake, max_steering)
        if next_waypoint.is_junction and ignore_junction == False:
            if speed > 30.0:
                speed = 30.0
            control_signal = custom_controller.run_step(
                target_speed=speed, waypoint=next_waypoint)
        else:
            control_signal = custom_controller.run_step(
                target_speed=speed, waypoint=next_waypoint)
        vehicle_actor.apply_control(control_signal)

    def spectator_follow_vehicle(spectator: carla.Actor, spec_location: carla.Location, sensor_actor: carla.Actor, pitch: float = -50.0, yaw: float = 0.0, range: float = 30):
        '''
        spectator following the ego vehicle

        Params:
            spectator(carla.Actor): the spectator himself
            spec_location(carla.Location): the current Location of the spectator
            sensor_actor (carla.Actor): sensor attached to the vehicle as a spectator
            pitch (float -180.0 to 180.0): Angle in degrees for rotation around the pitch
            yaw (float -180.0 to 180.0): Angle in degrees for rotation around the yaw
            range(float): the height of the spectator (z-axis)

        Return:
            spec_location(carla.Location): the current location of the spectator
        '''
        transform = sensor_actor.get_transform()
        transform.rotation.pitch = transform.rotation.pitch + pitch
        transform.rotation.yaw = transform.rotation.yaw + yaw
        spectator.set_transform(carla.Transform(
            spec_location + carla.Location(z=range), transform.rotation))
        return sensor_actor.get_location()

    def wp_lane_left(vehicle_actor: carla.Actor, map: carla.Map):
        '''
        looks for waypoints to the left of the vehicle

        Params:
            vehicle_actor (carla.Actor): the vehicle from which the left waypoints are to be found
            map(carla.Map): the map in which the waypoints are to be found
        Return:
            waypoint(carla.Waypoint):the next waypoint ond the left lane
        '''
        waypoint = map.get_waypoint(vehicle_actor.get_location(
        ))
        left_wp = waypoint.get_left_lane()
        if left_wp != None:

            if (waypoint.lane_id < 0 and left_wp.lane_id < 0) or (waypoint.lane_id > 0 and left_wp.lane_id > 0):
                next_wp = left_wp.next(10)[0]
                return next_wp
        next_wp = waypoint.next(10)[0]
        return next_wp

    def wp_lane_right(vehicle_actor: carla.Actor, map: carla.Map):
        waypoint = map.get_waypoint(vehicle_actor.get_location(
        ))
        '''
        looks for waypoints to the right of the vehicle

        Params:
            vehicle_actor (carla.Actor): the vehicle from which the right waypoints are to be found
            map(carla.Map): the map in which the waypoints are to be found

        Return:
            waypoint(carla.Waypoint):the next waypoint ond the right lane
        '''
        right_wp = waypoint.get_right_lane()
        if right_wp != None:

            if (waypoint.lane_id < 0 and right_wp.lane_id < 0) or (waypoint.lane_id > 0 and right_wp.lane_id > 0):
                next_wp = right_wp.next(10)[0]
                return next_wp
        next_wp = waypoint.next(10)[0]

        return next_wp

    def wp_ahead(vehicle_actor: carla.Actor, map: carla.Map, distance: float = 5.0):
        '''
        waypoints that lie in front of the vehicle

        Params:
            vehicle_actor(carla.Actor): the vehicle from which the left waypoints are to be found
            map(carla.Map): the map in which the waypoints are to be found

        return:
            waypoint(carla.Waypoint): the next waypoint infront of the vehicle
        '''
        waypoint = map.get_waypoint(vehicle_actor.get_location(
        ))

        next_wp = waypoint.next(distance)[0]
        return next_wp

    def spawn_vehicle(actor_list: list, blueprint_library: carla.BlueprintLibrary, world: carla.World, vehicle_blueprint_name: str, vehicle_name: str, spawn_point: carla.Transform, color: str = '255,0,0'):
        '''
        Spawns a vehicle in the Carla Simulator

        Params:
            actor_list[]: list of all actors on client side
            blueprint_library(carla.BlueprintLibrary): the carla blueprint library from carla.World
            world(carla.World): world informations from the Carla Simulator
            vehicle_blueprint_name(str): name of the vehicle in the blueprint library
            vehicle_name(str): the name the vehicle has in the simulation
            spawn_point(carla.Transform): the transformation at which the vehicle appears
            color(str): the color of the vehicles as string ("r,g,b")

        return:
            vehicle(carla.Vehicle): the vehicle himself
        '''
        vehicle_blueprint = blueprint_library.filter(vehicle_blueprint_name)[
            0]
        vehicle_blueprint.set_attribute('role_name', vehicle_name)
        vehicle_blueprint.set_attribute('color', color)
        vehicle = world.spawn_actor(
            vehicle_blueprint, spawn_point)
        actor_list.append(vehicle)
        return vehicle

    def spawn_traffic(vehicle_list, spawnpoints, quantity: int = 10):
        '''
        spawn random traffic in the simulation that is driven by the autopilot

        Params:
            quantity(int): number of spawned vehicles
        '''
        for i in range(quantity):
            sp_pos = randint(0, len(spawnpoints) - 1)
            pos_vl = randint(0, len(vehicle_list) - 1)
            vehicle_bp = vehicle_list[pos_vl]
            spawnpoint = spawnpoints[sp_pos]
            spawnpoints.pop(sp_pos)
            vehicle_name = 'other' + str(i)
            vehicle = Agent.spawn_vehicle(
                vehicle_bp, vehicle_name, spawnpoint, '0,0,255')
            vehicle.set_autopilot()

    def spawn_obstacle(actor_list: list, blueprint_library: carla.BlueprintLibrary, world: carla.World, vehicle_blueprint_name: str, vehicle_name: str, spawn_point: carla.Transform):
        '''
        Spawns a vehicle in the Carla Simulator

        Params:
            actor_list[]: list of all actors on client side
            blueprint_library(carla.BlueprintLibrary): the carla blueprint library from carla.World
            world(carla.World): world informations from the Carla Simulator
            vehicle_blueprint_name(str): name of the vehicle in the blueprint library
            vehicle_name(str): the name the vehicle has in the simulation
            spawn_point(carla.Transform): the transformation at which the vehicle appears

        return:
            obstacle(carla.Actor): the obstacle himself
        '''
        obstacle_blueprint = blueprint_library.filter(vehicle_blueprint_name)[
            0]
        obstacle_blueprint.set_attribute('role_name', vehicle_name)
        obstacle = world.spawn_actor(
            obstacle_blueprint, spawn_point)
        actor_list.append(obstacle)
        return obstacle

    def set_blinker(vehicle: carla.Vehicle, side: str):
        '''
        Sets the turn signal of a vehicle

        Params:
            vehicle(carla.Vehicle): the vehicle for which the turn signal is to be set
            side(str): the side on which the flasher is to be set
        '''
        if side == "left":
            vehicle.set_light_state(carla.VehicleLightState.LeftBlinker)
        elif side == "right":
            vehicle.set_light_state(carla.VehicleLightState.RightBlinker)

    def set_blinker_off(vehicle: carla.Vehicle):
        '''
        set the turn signal off

        Params:
            vehicle(carla.Vehicle): the vehicle for which the turn signal is to be switched off
        '''
        vehicle.set_light_state(carla.VehicleLightState.NONE)

    def break_on_red_light(vehicle: carla.Vehicle):
        '''
        Brakes at red light and sets its state to green

        Params:
            vehicle(carla.Vehicle): The vehicle which is to set the traffic lights to green

        Return:
            Lightstate(bool): red = True green = False
        '''
        tfligth = vehicle.get_traffic_light()
        if tfligth != None:
            state = tfligth.state
            if state == carla.TrafficLightState.Red:
                tfligth.set_state(carla.TrafficLightState.Green)
                return True
        return False

    def draw_waypoints(world: carla.World, wps, color: carla.Color, life_time=100):
        '''
        Draw Waypoints from a list

        Params:
            world(carla.World): world informations from the Carla Simulator
            wps([carla.Waypoints]): list of waypoints to draw
            color(carla.Color): color of waypoints
            life_time: time the waypoints life in the world
        '''
        for waypoint in wps:
            world.debug.draw_string(waypoint.transform.location, 'O', draw_shadow=True,
                                    color=color, life_time=life_time,
                                    persistent_lines=True)

    def draw_waypoint(world: carla.World, wp, color: carla.Color, life_time=1):
        '''
        Draw a single Waypoint

        Params:
            world(carla.World): world informations from the Carla Simulator
            wp([carla.Waypoints]): a single waypoint
            color(carla.Color): color of waypoints
            life_time: time the waypoints life in the world
        '''
        world.debug.draw_string(wp.transform.location, 'O', draw_shadow=True,
                                color=color, life_time=life_time,
                                persistent_lines=True)

    def get_next_waypoints(vehicle_actor: carla.Actor, map: carla.Map, distance: float = 5.0, quantity: int = 5, wp_list: List[carla.Waypoint] = []) -> List[carla.Waypoint]:
        '''
        get the next n waypoints

        Params:
            vehicle_Actor(carla.Actor): the vehicle Actor
            map(carla.Map): map informations from the Carla Simulator
            distance(float): distance beetween the waypoints
            quantity(int): the number of generating waypoints

        Return:
            wp_list([carla.Waypoints]): list of Waypoints
        '''

        waypoint = map.get_waypoint(vehicle_actor.get_location(
        ), lane_type=carla.LaneType.Driving)
        next_wp = waypoint.next(distance)[0]
        if wp_list != []:
            wp_list = Agent.stabalize_waypoints_at_junction(
                wp_list, distance)
            return wp_list

        wp_list.clear()
        for i in range(quantity):

            wp_list.append(next_wp)
            next_wp = next_wp.next(distance)[0]
        return wp_list

    def stabalize_waypoints_at_junction(wp_list: List[carla.Waypoint] = [], distance: float = 5.0):
        '''
        Stabilize the waypoints while the vehicle is at an intersection

        Params:
            wp_list(List[carla.Waypoint]): waypoints from current trajectory 
            distance(float): distance between the waypoints 
        Return:
            wp_list([carla.Waypoints]): waypoints new trajectory
        '''
        if wp_list != []:
            wp_list.pop(0)
            next_wp = wp_list[-1].next(distance)[0]
            wp_list.append(next_wp)
        return wp_list

    def get_next_waypoints_traject(vehicle: carla.Actor, next_waypoint: carla.Waypoint, map: carla.Map, distance: float = 5.0, quantity: int = 5) -> List[carla.Waypoint]:
        '''
        get the next n waypoints

        Params:
            vehicle_Actor(carla.Actor): the vehicle Actor
            map(carla.Map): map informations from the Carla Simulator
            distance(float): distance beetween the waypoints
            quantity(int): the number of generating waypoints

        Return:
            wp_list([carla.Waypoints]): list of Waypoints
        '''

        waypoint = map.get_waypoint(next_waypoint.transform.location)
        wp_list = []
        wp_list.append(map.get_waypoint(vehicle.get_location()))
        next_wp = waypoint.next(distance)[0]
        for i in range(quantity):
            wp_list.append(next_wp)
            next_wp = next_wp.next(distance)[0]
        return wp_list

    def get_current_waypoint(vehicle_actor: carla.Actor, map: carla.Map) -> carla.Waypoint:
        '''
        get the waypoint at which the vehicle is currently located

        Params:
            vehicle_Actor(carla.Actor): the vehicle Actor
            map(carla.Map): map informations from the Carla Simulator

        Return:
            waypoint carla.Waypoint): the waypoint at which the vehicle is currently located
        '''
        waypoint = map.get_waypoint(vehicle_actor.get_location(
        ))
        return waypoint

    def draw_traject(world: carla.World, vehicle: carla.Actor,  wp_list, color: carla.Color = carla.Color(0, 76, 0), z=0.0):
        '''
        Draw the trajectory in CARLA

        Params:
            world(carla.World): world informations from the Carla Simulator
            vehicle_Actor(carla.Actor): the vehicle Actor
            map(carla.Map): map informations from the Carla Simulator
            distance(float): distance beetween the waypoints
            quantity(int): the number of generating waypoints

        Return:
            wp_list([carla.Waypoints]): list of Waypoints
        '''
        start = vehicle.get_transform().transform(carla.Location(0.0, 0, 0))
        start = carla.Location(start.x, start.y, start.z)
        for i in range(0, len(wp_list) - 1):
            world.debug.draw_line(
                start, wp_list[i+1].transform.location + carla.Location(0, 0, z), life_time=0.05, thickness=0.25, color=color)
            start = wp_list[i+1].transform.location

    def get_trajectory_planed_lane_change(vehicle: carla.Actor, next_waypoint: carla.Waypoint, map: carla.Map, start: carla.Location, distance: float = 5.0, quantity: int = 5, direction: str = "left"):
        '''
        Generate a trajectory for a planned lane change

        Params:
            vehicle_Actor(carla.Actor): the vehicle Actor
            next_waypoint(carla.Waypoint): the next waypoint that the vehicle intends to reach
            map(carla.Map): map informations from the Carla Simulator
            start(carla.Location): the point at which the lane change is intended to occur
            distance(float): distance beetween the waypoints
            quantity(int): the number of generating waypoints
            direction(str): the direction in which the lane change is to be executed
        Return:
            wp_list([carla.Waypoints]): list of Waypoints
        '''
        waypoint = map.get_waypoint(next_waypoint.transform.location)
        wp_list = []
        wp_list.append(map.get_waypoint(vehicle.get_location()))
        next_wp = waypoint.next(distance)[0]
        for i in range(quantity):
            wp_list.append(next_wp)
            next_wp = next_wp.next(distance)[0]
            if next_wp.transform.location.x > start.x and next_wp.transform.location.x < start.x + distance + 5.0:
                if direction == "left":
                    left_wp = next_wp.get_left_lane()
                    next_wp = left_wp.next(5)[0]
                elif direction == "right":
                    right_wp = next_wp.get_right_lane()
                    next_wp = right_wp.next(5)[0]
        return wp_list
