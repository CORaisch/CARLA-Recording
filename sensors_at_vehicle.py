import carla
import random
import math
import pygame

def get_fov_from_fx(image_widht, fx):
    return (2.0 * math.atan(image_widht/(2.0*fx))) * 180.0 / math.pi

def process_rgb_camera(data):
    # print some information about captured data
    print("RGB Camera captured frame #{0}: Timestamp: {1}, Pose: {2}".format(data.frame, data.timestamp, data.transform))
    # save image to disk
    path = "out/im_" + str(data.timestamp) + ".png"
    data.save_to_disk(path, color_converter=carla.ColorConverter.Raw)

def main_loop(world):
    try:
        # loop endlessly and update specator data
        clock = pygame.time.Clock()
        while True:
            # limit iterations to 60 Hz
            clock.tick_busy_loop(60)
            # get snapshot of current world state
            world_snap = world.get_snapshot()
            # get snapshot of spectator actor
            spec_snap = world_snap.find(world.get_spectator().id)
            # get current pose of spectator
            spec_T = spec_snap.get_transform()
            # print spectator pose
            # print("location: {0} | rotation: {1}".format(spec_T.location, spec_T.rotation))
    finally:
        # NOTE if main loop related stuff needs to be removed afterwards, then do this here
        pass

def main():
    # connect to server at 134.2.178.201 (ZDV net) at port 2000
    client = carla.Client('134.2.178.201', 2000)
    client.set_timeout(10.0) # set 10 sec network timeout

    # retrieve world from server
    world = client.get_world()

    # adjust traffic light timings (too long red times at default^^)
    for traffic_light in world.get_actors().filter('traffic.traffic_light'):
        traffic_light.set_red_time(2.0)

    # retrieve spectator actor from world
    spec    = world.get_spectator()
    spec_id = spec.id

    ## spawn vehicle to which the camera should be attached
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
        raise RuntimeError("failed spawning vehicle")


    # setup rgb camera sensor
    rgb_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
    rgb_blueprint.set_attribute('image_size_x', '1920')
    rgb_blueprint.set_attribute('image_size_y', '1080')
    rgb_blueprint.set_attribute('fov', '110') # 110 deg field of view
    rgb_blueprint.set_attribute('sensor_tick', '0.2') # capture 20 frame per second
    car_bb = car_actor.bounding_box
    cam_rel_transform = carla.Transform(carla.Location(x=car_bb.extent.x, y=car_bb.extent.y, z=car_bb.extent.z*2.0+0.5)) # transform of camera relative to attached actor
    rgb_sensor = world.spawn_actor(rgb_blueprint, cam_rel_transform, attach_to=car_actor) # attach rgb camera to spectator
    rgb_sensor.listen(lambda data: process_rgb_camera(data))

    try:
        main_loop(world)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    finally:
        # remove sensor
        rgb_sensor.destroy()
        # remove vehicle
        client.apply_batch([carla.command.DestroyActor(car_actor)])


if __name__ == "__main__":
    main()
