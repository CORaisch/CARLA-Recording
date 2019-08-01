#!/usr/bin/env python

# Copyright (c) 2019 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import glob
import os
import sys
import argparse
import pygame
import random
import math
import numpy as np
import carla
import queue
import threading

## globals
is_running = True
sequence_buffer = queue.Queue()

# NOTE taken from PythonAPI/examples/synchronous_mode.py
class CarlaSyncMode(object):
    """
    Context manager to synchronize output from different sensors. Synchronous
    mode is enabled as long as we are inside this context

        with CarlaSyncMode(world, sensors) as sync_mode:
            while True:
                data = sync_mode.tick(timeout=1.0)

    """

    def __init__(self, world, sensors, **kwargs):
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

        def make_queue_snapshot(register_event):
            q = queue.Queue()
            register_event(q.put)
            self._queues.append((q, 'carla.WorldSnapshot'))

        def make_queue_sensor(sensor, register_event):
            q = queue.Queue()
            register_event(q.put)
            # self._queues.append((q, sensor.type_id))
            self._queues.append((q, sensor[1]))

        make_queue_snapshot(self.world.on_tick)
        for sensor in self.sensors:
            # make_queue_sensor(sensor, sensor.listen)
            make_queue_sensor(sensor, sensor[0].listen)
        return self

    def tick(self, timeout):
        self.frame = self.world.tick()
        # q[0] == queue, q[1] == sensor_id
        data = [(self._retrieve_data(q[0], timeout), q[1]) for q in self._queues]
        assert all(x[0].frame == self.frame for x in data)
        return data # -> data = [(world_snapshot, "carla.WorldSnapshot"), (data_sensor_1, type_sensor_1), ..., (data_sensor_n, type_sensor_n)]

    def __exit__(self, *args, **kwargs):
        self.world.apply_settings(self._settings)

    def _retrieve_data(self, sensor_queue, timeout):
        while True:
            data = sensor_queue.get(timeout=timeout)
            if data.frame == self.frame:
                return data

def should_quit():
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            return True
        elif event.type == pygame.KEYUP:
            if event.key == pygame.K_ESCAPE:
                return True
    return False

def raw_to_np_array(raw):
    arr = np.frombuffer(raw.raw_data, dtype=np.dtype("uint8"))
    arr = np.reshape(arr, (raw.height, raw.width, 4))
    arr = arr[:, :, :3]
    arr = arr[:, :, ::-1]
    return arr

def draw_image(surface, image, blend=False):
    array = raw_to_np_array(image)
    image_surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
    if blend:
        image_surface.set_alpha(100)
    surface.blit(image_surface, (0, 0))

# NOTE assuming that all sensors have same size
def display_sensors(surface, width, height, data):
    arr = np.zeros((2*height, 2*width, 3))
    # retrieve sensors
    # TODO make function more generic
    rgb_l = get_sensor(data, 'sensor.camera.rgb.left')
    rgb_r = get_sensor(data, 'sensor.camera.rgb.right')
    depth = get_sensor(data, 'sensor.camera.depth.left')
    semseg = get_sensor(data, 'sensor.camera.semantic_segmentation.left')
    if rgb_l:
        arr[:height, :width, :] = raw_to_np_array(rgb_l[0])
    if rgb_r:
        arr[:height, width:, :] = raw_to_np_array(rgb_r[0])
    if depth:
        depth[0].convert(carla.ColorConverter.LogarithmicDepth)
        arr[height:, :width, :] = raw_to_np_array(depth[0])
    if semseg:
        semseg[0].convert(carla.ColorConverter.CityScapesPalette)
        arr[height:, width:, :] = raw_to_np_array(semseg[0])
    # render to display
    image_surface = pygame.surfarray.make_surface(arr.swapaxes(0, 1))
    surface.blit(image_surface, (0, 0))

def get_font():
    fonts = [x for x in pygame.font.get_fonts()]
    default_font = 'ubuntumono'
    font = default_font if default_font in fonts else fonts[0]
    font = pygame.font.match_font(font)
    return pygame.font.Font(font, 14)

def get_sensor(data, sensor_type):
    return [x[0] for x in data if x[1]==sensor_type]

def spawn_camera_sensor(sensor_type, world, widht, height, fov, parent_actor, rel_transform, baseline):
    # set camera type parameter
    sensor_position = sensor_type.split(sep='.')[-1] # NOTE assuming last string is {left|right}
    camera_type = 'sensor.camera.' + sensor_type[:-(len(sensor_position)+1)]
    # set relative transform
    if sensor_position == 'right':
            rel_transform.location.y += baseline;
    # set camera sensor type
    rgb_blueprint = world.get_blueprint_library().find(str(camera_type))
    # set image size
    rgb_blueprint.set_attribute('image_size_x', str(widht))
    rgb_blueprint.set_attribute('image_size_y', str(height))
    # set field of view (in deg)
    rgb_blueprint.set_attribute('fov', str(fov))
    # set frame capturing rate
    rgb_blueprint.set_attribute('sensor_tick', str(0.0))
    # set special attributes for rgb cameras
    if camera_type == 'sensor.camera.rgb':
        # set motion blur parameters
        # TODO find out good value for motion blur intensity
        rgb_blueprint.set_attribute('motion_blur_intensity', '0.25') # 0 == off | 1 == max | 0.45 == default
        rgb_blueprint.set_attribute('motion_blur_max_distortion', '0.35') # in percentage of screen width | 0 == off
        rgb_blueprint.set_attribute('motion_blur_min_object_screen_size', '0.1') # percentage of screen widht objects must have for motion blur
        # set target gamma value of camera
        rgb_blueprint.set_attribute('gamma', '2.2')
    # spawn sensor and attach it to actor
    return world.spawn_actor(rgb_blueprint, rel_transform, attach_to=parent_actor), camera_type+'.'+sensor_position

def disk_writer_loop():
    import time
    global is_running
    global sequence_buffer
    sequence_id = 0
    while is_running:
        while sequence_buffer.qsize():
            if not is_running:
                print(".", end='', flush=True)
            save_measurements_to_disk(sequence_id, sequence_buffer.get())
            sequence_id += 1

def save_measurements_to_disk(sequence_id, measurements):
    # prepare paths
    base_path = "raw/"
    filename  = "images/" + str(sequence_id).zfill(10)
    # iterate sensors and store their outputs into appropriate directory
    gt_transform = None
    for measurement in measurements:
        sensor_position = measurement[1].split(sep='.')[-1]
        sensor_type = measurement[1][:-(len(sensor_position)+1)]
        if sensor_type == 'sensor.camera.rgb':
            if gt_transform == None and sensor_position == 'left':
                gt_transform = measurement[0].transform
            measurement[0].save_to_disk(base_path + "rgb/" + sensor_position + "/" + filename + ".png")
        elif sensor_type == 'sensor.camera.depth':
            if gt_transform == None and sensor_position == 'left':
                gt_transform = measurement[0].transform
            # TODO logarithmic depth only for debugging, later save as raw !!
            measurement[0].save_to_disk(base_path + "depth/" + sensor_position + "/" + filename + ".png", color_converter=carla.ColorConverter.LogarithmicDepth)
        elif sensor_type == 'sensor.camera.semantic_segmentation':
            if gt_transform == None and sensor_position == 'left':
                gt_transform = measurement[0].transform
            # TODO cityscapes palette only for debugging, later save as raw !!
            measurement[0].save_to_disk(
                base_path + "semantic_segmentation/" + sensor_position + "/" + filename + ".png", color_converter=carla.ColorConverter.CityScapesPalette)
        else:
            # TODO make assert or something else that lets the program stop instead in this case of simple warning
            print("undefined sensor requested: ", measurement[1])
    # store left camera poses
    if gt_transform != None:
        ## NOTE rotations according to: https://carla.readthedocs.io/en/latest/python_api/#carla.Rotation
        y = gt_transform.rotation.yaw
        p = gt_transform.rotation.pitch
        r = gt_transform.rotation.roll
        ## store rotation as rotation matrix
        def c(x):
            return math.cos(x)
        def s(x):
            return math.sin(x)
        # using: http://planning.cs.uiuc.edu/node102.html -> gt_rotation = R_z(yaw)*R_y(pitch)*R_x(roll)
        gt_pose  = str(c(y)*c(p)) + " " + str(c(y)*s(p)*s(r)-s(y)*c(r)) + " " + str(c(y)*s(p)*c(r)+s(y)*s(r)) + " " + str(gt_transform.location.x) + " "
        gt_pose += str(s(y)*c(p)) + " " + str(s(y)*s(p)*s(r)+c(y)*c(r)) + " " + str(s(y)*s(p)*c(r)-c(y)*s(r)) + " " + str(gt_transform.location.y) + " "
        gt_pose += str(-s(p))     + " " + str(c(p)*s(r))                + " " + str(c(p)*c(r))                + " " + str(gt_transform.location.z) + "\n"
        # write to file
        with open(base_path + "poses.txt", "a") as poses_file:
            poses_file.write(gt_pose)
    else:
        print("no valid sensor attached for GT poses")

def get_fov_from_fx(image_widht, fx):
    return (2.0 * math.atan(image_widht/(2.0*fx))) * 180.0 / math.pi

def main():
    ## init argument parser
    argparser = argparse.ArgumentParser(description="Synchronously capture arbitrary sensors while they are attached to a vehicle driving on a random route. Following sensors are supported: RGB, Depth, Semantic Segmentation. Following sensors will be captured by default: Stereo RGB, Depth, Semantic Segmentation. The program outputs the images of the sensors, the ground truth poses and the intrinsics of the cameras.")
    # add arguments
    argparser.add_argument('--sensor_width', '-width', type=int, default=800, help="set width for all cameras")
    argparser.add_argument('--sensor_height', '-height', type=int, default=600, help="set height for all cameras")
    argparser.add_argument('--fov', '-fov', type=float, default=90.0, help="set FoV for all cameras")
    argparser.add_argument('--baseline', '-baseline', type=float, default=0.5, help="set baseline for rectified stereo setup (in meter)")
    argparser.add_argument('--sensors', '-sensors', type=str, default=[], nargs='*', help="add one of the following sensors: 'rgb.{left|right}', 'depth.{left|right}', 'semantic_segmentation.{left|right}'")
    argparser.add_argument('--left_rel_location', '-left_rel_location', type=float, default=[], nargs=3, help="set relative loation of left camera")
    argparser.add_argument('--fps', '-fps', type=float, default=10.0, help="set captuing rate of sensors (WARNING don't set value below 10 fps, its not yet supported by CARLA)")
    # finally parse arguments
    args = argparser.parse_args()

    ## init pygame
    actor_list = []
    pygame.init()

    display = pygame.display.set_mode(
        (args.sensor_width * 2, args.sensor_height * 2),
        pygame.HWSURFACE | pygame.DOUBLEBUF)
    font = get_font()
    clock = pygame.time.Clock()

    client = carla.Client('134.2.178.201', 2000)
    client.set_timeout(10.0)

    world = client.get_world()

    settings_w = world.get_settings()

    try:
        # set random spawning location of sensor carrying vehicle
        m = world.get_map()
        start_pose = random.choice(m.get_spawn_points())
        waypoint = m.get_waypoint(start_pose.location)

        ## setup world properties
        # set traffic light timings
        for traffic_light in world.get_actors().filter('traffic.traffic_light'):
            traffic_light.set_red_time(1.0)
            traffic_light.set_yellow_time(0.5)
            traffic_light.set_green_time(2.0)
        # TODO set random weather state of world

        # get list of actor blueprints
        blueprint_library = world.get_blueprint_library()

        # spawn vehicle to which sensors should be attached
        car_blueprints = [x for x in world.get_blueprint_library().filter('vehicle.*') if int(x.get_attribute('number_of_wheels')) == 4]
        vehicle = world.spawn_actor(random.choice(car_blueprints), start_pose)
        actor_list.append(vehicle)
        vehicle.set_simulate_physics(True)
        vehicle.set_autopilot(True)

        ## spawn sensors
        sensor_list = []
        # define left and right camera poses relative to vehicle
        if not args.left_rel_location:
            vehicle_bb = vehicle.bounding_box
            cam_rel_transform_l = carla.Transform(carla.Location(x=vehicle_bb.extent.x, y=vehicle_bb.extent.y-args.baseline/2.0, z=vehicle_bb.extent.z*2.0+0.5))
        else:
            pos_x, pos_y, pos_z = args.left_rel_location
            cam_rel_transform_l = carla.Transform(carla.Location(x=pos_x, y=pos_y-args.baseline/2.0, z=pos_z))

        if not args.sensors:
            # spawn left stereo rgb camera
            camera_rgb_left, sensor_id = spawn_camera_sensor(
                'rgb.left', world, args.sensor_width, args.sensor_height, args.fov, vehicle, cam_rel_transform_l, args.baseline
            )
            actor_list.append(camera_rgb_left)
            sensor_list.append((camera_rgb_left, sensor_id))
            # spawn right stereo rgb camera
            camera_rgb_right, sensor_id = spawn_camera_sensor(
                'rgb.right', world, args.sensor_width, args.sensor_height, args.fov, vehicle, cam_rel_transform_l, args.baseline
            )
            actor_list.append(camera_rgb_right)
            sensor_list.append((camera_rgb_right, sensor_id))
            # spawn depth camera at left camera pose
            camera_depth, sensor_id = spawn_camera_sensor(
                'depth.left', world, args.sensor_width, args.sensor_height, args.fov, vehicle, cam_rel_transform_l, args.baseline
            )
            actor_list.append(camera_depth)
            sensor_list.append((camera_depth, sensor_id))
            # spawn semantic segmentation camera at left camera pose
            camera_semseg, sensor_id = spawn_camera_sensor(
                'semantic_segmentation.left', world, args.sensor_width, args.sensor_height, args.fov, vehicle, cam_rel_transform_l, args.baseline
            )
            actor_list.append(camera_semseg)
            sensor_list.append((camera_semseg, sensor_id))
        else:
            for sensor in args.sensors:
                sensor_actor, sensor_id = spawn_camera_sensor(
                    sensor, world, args.sensor_width, args.sensor_height, args.fov, vehicle, cam_rel_transform_l, args.baseline
                )
                actor_list.append(sensor_actor)
                sensor_list.append((sensor_actor, sensor_id))

        ## prepare buffers for sensor measurements
        # sequence_count  = 0
        global sequence_buffer
        # invoke disk writer thread
        disk_writer_thread = threading.Thread(target=disk_writer_loop)
        # create a synchronous mode context.
        with CarlaSyncMode(world, sensor_list, fps=args.fps) as sync_mode:
            # start disk writer thread
            disk_writer_thread.start()
            while True:
                if should_quit():
                    return
                clock.tick()

                ## advance the simulation and wait for the data
                data = sync_mode.tick(timeout=2.0)

                ## save data to disk
                # push measurements into buffer TODO validate threadsafeness of queue.Queue
                sequence_buffer.put(data[1:]) # don't push snapshot

                ## compute simulated fps (for verification)
                snapshot = get_sensor(data, 'carla.WorldSnapshot')[0]
                fps = round(1.0 / snapshot.timestamp.delta_seconds)

                ## visualize sensor measurements
                display_sensors(display, args.sensor_width, args.sensor_height, data)

                # draw left rgb and semantic segmentation overlayed (to verify if left frames are synchronized)
                # draw_image(display, im_rgb_l)
                # draw_image(display, im_semseg, blend=True)

                # draw simulation timings
                display.blit(font.render('% 5d FPS (real)' % clock.get_fps(), True, (255, 255, 255)), (8, 10))
                display.blit(font.render('% 5d FPS (simulated sensor capturing rate)' % fps, True, (255, 255, 255)), (8, 28))
                pygame.display.flip()

    finally:
        # stop and wait for disk writer thread
        print("\nwait till remaining sensor measurements are written to disk", end='', flush=True)
        global is_running
        is_running = False
        disk_writer_thread.join()
        print(" done")
        # destroy sequence buffer
        del sequence_buffer # TODO ensure that all measurements are written to disk before deletion
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
