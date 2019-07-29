#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys
import pygame
import random
import numpy as np
import carla
import queue


# NOTE taken from PythonAPI/examples/synchronous_mode.py
class CarlaSyncMode(object):
    """
    Context manager to synchronize output from different sensors. Synchronous
    mode is enabled as long as we are inside this context

        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)

    """

    def __init__(self, world, *sensors, **kwargs):
        self.world = world
        self.sensors = sensors
        self.frame = None
        self.delta_seconds = 1.0 / kwargs.get('fps', 20)
        self._queues = []
        self._settings = None

    def __enter__(self):
        self._settings = self.world.get_settings()
        self.frame = self.world.apply_settings(carla.WorldSettings(
            no_rendering_mode=False,
            synchronous_mode=True,
            fixed_delta_seconds=self.delta_seconds))

        def make_queue(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append(q)

        make_queue(self.world.on_tick)
        for sensor in self.sensors:
            make_queue(sensor.listen)
        return self

    def tick(self, timeout):
        self.frame = self.world.tick()
        data = [self._retrieve_data(q, timeout) for q in self._queues]
        assert all(x.frame == self.frame for x in data)
        return data

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data

def raw_to_np_array(raw):
    arr = np.frombuffer(raw.raw_data, dtype=np.dtype("uint8"))
    arr = np.reshape(arr, (raw.height, raw.width, 4))
    arr = arr[:, :, :3]
    arr = arr[:, :, ::-1]
    return arr

def draw_image(surface, image, blend=False):
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    array = array[:, :, :3]
    array = array[:, :, ::-1]
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    if blend:
        image_surface.set_alpha(100)
    surface.blit(image_surface, (0, 0))

def display_sensors(surface, rgb_l, rgb_r, depht, semseg):
    arr = np.zeros((2*rgb_l.height, 2*rgb_l.width, 3)) # NOTE assuming that all sensors have same size
    arr[:rgb_l.height, :rgb_l.width, :] = raw_to_np_array(rgb_l)
    arr[:rgb_l.height, rgb_l.width:, :] = raw_to_np_array(rgb_r)
    arr[rgb_l.height:, :rgb_l.width, :] = raw_to_np_array(depht)
    arr[rgb_l.height:, rgb_l.width:, :] = raw_to_np_array(semseg)
    # render to display
    image_surface = pygame.surfarray.make_surface(arr.swapaxes(0, 1))
    surface.blit(image_surface, (0, 0))

def get_font():
    fonts = [x for x in pygame.font.get_fonts()]
    default_font = 'ubuntumono'
    font = default_font if default_font in fonts else fonts[0]
    font = pygame.font.match_font(font)
    return pygame.font.Font(font, 14)


def should_quit():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_ESCAPE:
                return True
    return False


def main():
    actor_list = []
    pygame.init()

    sensor_height = 600; sensor_width = 800;

    display = pygame.display.set_mode(
        (sensor_width * 2, sensor_height * 2),
        pygame.HWSURFACE | pygame.DOUBLEBUF)
    font = get_font()
    clock = pygame.time.Clock()

    client = carla.Client('134.2.178.201', 2000)
    client.set_timeout(10.0)

    world = client.get_world()

    settings_w = world.get_settings()

    try:
        m = world.get_map()
        start_pose = random.choice(m.get_spawn_points())
        waypoint = m.get_waypoint(start_pose.location)

        blueprint_library = world.get_blueprint_library()

        car_blueprints = [x for x in world.get_blueprint_library().filter('vehicle.*') if int(x.get_attribute('number_of_wheels')) == 4]
        vehicle = world.spawn_actor(random.choice(car_blueprints), start_pose)
        actor_list.append(vehicle)
        vehicle.set_simulate_physics(True)
        vehicle.set_autopilot(True)

        ## spawn sensors
        # define left and right camera poses relative to vehicle
        baseline = 0.5 # baseline in m ? (TODO verify CARLA distance units)
        vehicle_bb = vehicle.bounding_box
        cam_rel_transform_l = carla.Transform(carla.Location(x=vehicle_bb.extent.x, y=vehicle_bb.extent.y-baseline/2.0, z=vehicle_bb.extent.z*2.0+0.5))
        cam_rel_transform_r = carla.Transform(carla.Location(x=vehicle_bb.extent.x, y=vehicle_bb.extent.y+baseline/2.0, z=vehicle_bb.extent.z*2.0+0.5))

        # TODO write function to spawn camera like sensors (see sensors_at_vehicle.py)
        # TODO modify attributes of sensor blueprints

        # spaw left stereo rgb camera
        camera_rgb_left = world.spawn_actor(
            blueprint_library.find('sensor.camera.rgb'),
            cam_rel_transform_l,
            attach_to=vehicle)
        actor_list.append(camera_rgb_left)

        # spaw right stereo rgb camera
        camera_rgb_right = world.spawn_actor(
            blueprint_library.find('sensor.camera.rgb'),
            cam_rel_transform_r,
            attach_to=vehicle)
        actor_list.append(camera_rgb_right)

        # spawn depth camera at left camera pose
        camera_depth = world.spawn_actor(
            blueprint_library.find('sensor.camera.depth'),
            cam_rel_transform_l,
            attach_to=vehicle)
        actor_list.append(camera_depth)

        # spawn semantic segmentation camera at left camera pose
        camera_semseg = world.spawn_actor(
            blueprint_library.find('sensor.camera.semantic_segmentation'),
            cam_rel_transform_l,
            attach_to=vehicle)
        actor_list.append(camera_semseg)

        # Create a synchronous mode context.
        with CarlaSyncMode(world, camera_rgb_left, camera_rgb_right, camera_depth, camera_semseg, fps=10) as sync_mode:
            while True:
                if should_quit():
                    return
                clock.tick()

                ## advance the simulation and wait for the data
                snapshot, im_rgb_l, im_rgb_r, im_depth, im_semseg = sync_mode.tick(timeout=2.0)

                # TODO store measurements in buffer

                # TODO write measurements from buffer to disk multithreaded

                ## convert sensor representations for visualization
                # use logarithmic depth scaling for depth map
                im_depth.convert(carla.ColorConverter.LogarithmicDepth)
                # use CityScapes palette for semantic segmentation
                im_semseg.convert(carla.ColorConverter.CityScapesPalette)

                ## compute simulated fps (for verification)
                fps = round(1.0 / snapshot.timestamp.delta_seconds)

                # draw images seperatley
                display_sensors(display, im_rgb_l, im_rgb_r, im_depth, im_semseg)

                # draw left rgb and semantic segmentation overlayed
                # draw_image(display, im_rgb_l)
                # draw_image(display, im_semseg, blend=True)

                # draw simulation timings
                display.blit(font.render('% 5d FPS (real)' % clock.get_fps(), True, (255, 255, 255)), (8, 10))
                display.blit(font.render('% 5d FPS (simulated sensor capturing rate)' % fps, True, (255, 255, 255)), (8, 28))
                pygame.display.flip()

    finally:
        # destroy actors and sensors
        print('destroying actors and sensors.')
        for actor in actor_list:
            actor.destroy()

        pygame.quit()
        print('done.')


if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
