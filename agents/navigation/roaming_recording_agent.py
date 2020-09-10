#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" This module implements an agent that roams around a track following random waypoints and avoiding other vehicles.
The agent also responds to traffic lights. """

from agents.navigation.agent import Agent, AgentState
from agents.navigation.local_planner import LocalPlanner, RoadOption

def read_waypoint_history(_carla_map, _world_name, _history_file):
    import json, carla

    def dict2wp(d):
        return (_carla_map.get_waypoint(carla.Location(x=d['x'], y=d['y'], z=d['z'])), RoadOption(d['opt']))

    def dict2tf(d):
        loc = carla.Location(x=d['x'], y=d['y'], z=d['z'])
        rot = carla.Rotation(roll=d['roll'], pitch=d['pitch'], yaw=d['yaw'])
        return carla.Transform(location=loc, rotation=rot)

    with open(_history_file, 'r') as f:
        data = json.load(f)
    world_name = data['world']
    assert world_name == _world_name, 'attempt to load waypoints for world {}, but they have been recorded on {}'.format(_world_name, world_name)
    init_tf = dict2tf(data['initial_tf'])
    n = data['n_waypoints']
    history = [ dict2wp(data[str(i)]) for i in range(n) ]
    return init_tf, history

class RoamingRecordingAgent(Agent):
    """
    RoamingRecordingAgent implements a basic agent that navigates scenes making random
    choices when facing an intersection. If 'record_file'
    is set the agent will save all reached waypoints at the given path. If 'global_plan' is set
    the agent will follow the waypoint trajectory given in the plan.

    This agent respects traffic lights and other vehicles. Traffic lights will be ignored if
    'ignore_traffic_lights' is set.
    """

    def __init__(self, vehicle, initial_tf, world_name, ignore_traffic_lights=False, record_file=None, global_plan=None, fps=10.0):
        """

        :param vehicle: actor to apply to local planner logic onto
        """
        super(RoamingRecordingAgent, self).__init__(vehicle)
        self._proximity_threshold = 10.0  # meters
        self._state = AgentState.NAVIGATING
        self._ignore_traffic_lights = ignore_traffic_lights
        self._record_file = record_file
        self._initial_transform = initial_tf
        self._world_name = world_name
        self._local_planner = LocalPlanner(self._vehicle, recording=bool(self._record_file), fps=fps)
        if global_plan:
            self._local_planner.set_global_plan(global_plan)

    def run_step(self, debug=False):
        """
        Execute one step of navigation.
        :return: carla.VehicleControl
        """

        # is there an obstacle in front of us?
        hazard_detected = False

        # retrieve relevant elements for safe navigation, i.e.: traffic lights
        # and other vehicles
        actor_list = self._world.get_actors()
        vehicle_list = actor_list.filter("*vehicle*")
        lights_list = actor_list.filter("*traffic_light*")

        # check possible obstacles
        vehicle_state, vehicle = self._is_vehicle_hazard(vehicle_list)
        if vehicle_state:
            if debug:
                print('!!! VEHICLE BLOCKING AHEAD [{}])'.format(vehicle.id))
            self._state = AgentState.BLOCKED_BY_VEHICLE
            hazard_detected = True

        # check for the state of the traffic lights
        if not self._ignore_traffic_lights:
            light_state, traffic_light = self._is_light_red(lights_list)
            if light_state:
                if debug:
                    print('=== RED LIGHT AHEAD [{}])'.format(traffic_light.id))
                self._state = AgentState.BLOCKED_RED_LIGHT
                hazard_detected = True

        if hazard_detected:
            control = self.emergency_stop()
        else:
            self._state = AgentState.NAVIGATING
            # update current speed limit
            target_speed = self._vehicle.get_speed_limit()
            if self._map.name == 'Town05': # limit speed for certain maps
                target_speed = min(target_speed, 60.0)
            if self._map.name == 'Town04' or self._map.name == 'Town06':
                target_speed = min(target_speed, 50.0)
            self._local_planner.set_speed(target_speed)
            # compute movement control vector
            control = self._local_planner.run_step()

        return control

    def done(self):
        return self._local_planner.done()

    def save_waypoint_history(self):
        def wp2dict(wp):
            loc = wp[0].transform.location; opt = wp[1].value
            return { 'x' : loc.x, 'y' : loc.y, 'z' : loc.z, 'opt' : opt }

        def tf2dict(tf):
            loc, rot = tf.location, tf.rotation;
            return { 'x' : loc.x, 'y' : loc.y, 'z' : loc.z, 'roll' : rot.roll, 'pitch' : rot.pitch, 'yaw' : rot.yaw }

        if self._record_file:
            # convert history dict
            history = self._local_planner.get_waypoint_history()
            n = len(history)
            data = {}
            data['world'] = self._world_name
            data['initial_tf'] = tf2dict(self._initial_transform)
            data['n_waypoints'] = n
            # TODO save map name
            for i in range(n):
                data[str(i)] = wp2dict(history[i])
            # write history to JSON
            import json
            with open(self._record_file, 'w') as f:
                json.dump(data, f)
        else:
            raise Exception('try to save wayoint history but none was recorded.')

