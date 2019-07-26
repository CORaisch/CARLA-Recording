import carla
import random
import math

## globals
rgb_sensor_left_framecount = 0

def get_fov_from_fx(image_widht, fx):
    return (2.0 * math.atan(image_widht/(2.0*fx))) * 180.0 / math.pi

def spawn_camera_sensor(camera_type, world, im_width, im_height, fov, fps, actor_to_attach_at, rel_transform):
    # set camera sensor type
    rgb_blueprint = world.get_blueprint_library().find(str(camera_type))
    # set image size
    rgb_blueprint.set_attribute('image_size_x', str(im_width))
    rgb_blueprint.set_attribute('image_size_y', str(im_height))
    # set field of view (in deg)
    rgb_blueprint.set_attribute('fov', str(fov))
    # set frame capturing rate
    sensor_tick = 1.0 / fps
    rgb_blueprint.set_attribute('sensor_tick', str(sensor_tick))
    # set special attributes for rgb cameras
    if camera_type == 'sensor.camera.rgb':
        # TODO find out good value for motion blur intensity
        rgb_blueprint.set_attribute('motion_blur_intensity', '0.25') # 0 == off | 1 == max | 0.45 == default
        # NOTE further attributes for rgb: motion_blur_max_distortion, motion_blur_min_object_screen_size, gamma, enable_postprocess_effects
    # spawn sensor and attach it to actor
    return world.spawn_actor(rgb_blueprint, rel_transform, attach_to=actor_to_attach_at)

# TODO investigate: sometimes when quitting out manually the last recorded images are not fully stored
def process_rgb_camera_left(data):
    global rgb_sensor_left_framecount
    # DEBUG print information about captured data
    print("RGB Camera captured frame #{0}: Timestamp: {1}, Pose: {2}".format(rgb_sensor_left_framecount, data.timestamp, data.transform))
    # save captured image to disk
    path = "raw/stereo/left/images/" + str(rgb_sensor_left_framecount).zfill(10) + ".png"
    rgb_sensor_left_framecount += 1
    data.save_to_disk(path, color_converter=carla.ColorConverter.Raw)
    # TODO append pose to file
    # TODO append timestamp to file

# the only reason for the main loop so far is that if the user breaks the loop via terminal then the vehicle should stop recording and despawn
def main_loop(world):
    # loop endlessly till user terminates manually
    while True:
        pass

def main():
    ## connect to server at 134.2.178.201 (ZDV net) at port 2000
    client = carla.Client('134.2.178.201', 2000)
    client.set_timeout(10.0) # set 10 sec network timeout

    ## retrieve world from server
    world = client.get_world()

    ## adjust traffic light timings
    # reduce red light timings
    for traffic_light in world.get_actors().filter('traffic.traffic_light'):
        # print("red time: {0}, yellow time: {1}, green time: {2}".format(traffic_light.get_red_time(), traffic_light.get_yellow_time(), traffic_light.get_green_time()))
        traffic_light.set_red_time(1.0)
        traffic_light.set_yellow_time(0.5)
        traffic_light.set_green_time(2.0)


    ## spawn vehicle to which the sensors should be attached
    # pick random spawn point from all possible spawn locations
    spawn_points = world.get_map().get_spawn_points()
    random.shuffle(spawn_points)
    spawn_point = spawn_points[0]
    # pick random car blueprint from all possible car blueprints
    car_blueprints = [x for x in world.get_blueprint_library().filter('vehicle.*') if int(x.get_attribute('number_of_wheels')) == 4]
    car_blueprint = random.choice(car_blueprints)
    # set role to autopilot
    car_blueprint.set_attribute('role_name', 'autopilot')
    # spawn vehicle and apply autopilot
    response = client.apply_batch_sync([carla.command.SpawnActor(car_blueprint, spawn_point).then(carla.command.SetAutopilot(carla.command.FutureActor, True))])
    if not response[0].error:
        car_id    = response[0].actor_id
        car_actor = world.get_actor(car_id)
    else:
        raise RuntimeError("failed spawning sensor carrying vehicle")

    ## setup left rgb camera sensor
    # set location relative to vehicle where the sensor should be attached
    car_bb = car_actor.bounding_box
    cam_rel_transform = carla.Transform(carla.Location(x=car_bb.extent.x, y=car_bb.extent.y, z=car_bb.extent.z*2.0+0.5))
    # spawn sensor
    rgb_sensor_left = spawn_camera_sensor('sensor.camera.rgb', world, 640, 480, 110.0, 20.0, car_actor, cam_rel_transform)
    # set sensor listener
    rgb_sensor_left.listen(lambda data: process_rgb_camera_left(data))

    try:
        main_loop(world)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    finally: # on exit: destroy sensors and despawn vehicle
        # remove sensor
        rgb_sensor_left.destroy()
        # remove vehicle
        client.apply_batch([carla.command.DestroyActor(car_actor)])


if __name__ == "__main__":
    main()
