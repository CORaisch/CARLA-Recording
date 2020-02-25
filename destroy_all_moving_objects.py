#!/home/claudio/Apps/anaconda3/bin/python

import carla

def main():
    # connect to server
    client = carla.Client('134.2.178.201', 2000)
    client.set_timeout(10.0) # set 10 sec network timeout

    # get world handle
    world = client.get_world()

    # destroy all existing actors that could potentially moving (i.e. vehicles and pedestrians)
    vehicle_response = client.apply_batch_sync([carla.command.DestroyActor(x) for x in world.get_actors().filter('vehicle.*')])
    walker_response  = client.apply_batch_sync([carla.command.DestroyActor(x) for x in world.get_actors().filter('walker.pedestrian.*')])

    # give final status report
    print("destroyed {0} vehicles and {1} walkers".format(len(vehicle_response), len(walker_response)))

if __name__ == "__main__":
    main()
