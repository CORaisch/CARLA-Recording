import carla
import pygame

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

    # retrieve spectator actor from world
    spec    = world.get_spectator()
    spec_id = spec.id

    # setup rgb camera sensor
    rgb_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
    rgb_blueprint.set_attribute('image_size_x', '1920')
    rgb_blueprint.set_attribute('image_size_y', '1080')
    rgb_blueprint.set_attribute('fov', '110') # 110 deg field of view
    rgb_blueprint.set_attribute('sensor_tick', '0.1') # capture 10 frame per second (10 Hz, like KITTI cameras)
    cam_rel_transform = carla.Transform(carla.Location(x=0.0,y=0.0,z=0.0)) # transform of camera relative to attached actor
    rgb_sensor = world.spawn_actor(rgb_blueprint, cam_rel_transform, attach_to=spec) # attach rgb camera to spectator
    rgb_sensor.listen(lambda data: process_rgb_camera(data))

    try:
        main_loop(world)
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')
    finally:
        # remove sensor
        rgb_sensor.destroy()


if __name__ == "__main__":
    main()
