import carla
import random

client = carla.Client('134.2.178.201', 2000)
client.set_timeout(10.0)

world = client.get_world()
m = world.get_map()
blueprint_library = world.get_blueprint_library()

# spawn first vehicle close to world origin
origin_transform = carla.Transform(carla.Location(x=100.4, y=129.7, z=1.0))
origin_blueprints = [x for x in world.get_blueprint_library().filter('vehicle.*') if int(x.get_attribute('number_of_wheels')) == 4]
origin_blueprint = random.choice(origin_blueprints)
origin_blueprint.set_attribute('color', '0,0,0') # set origin black
origin_actor = world.spawn_actor(origin_blueprint, origin_transform)
origin_actor.set_simulate_physics(True)

# spawn vehicle close to e1
e1_transform = carla.Transform(carla.Location(x=110.4, y=129.7, z=1.0))
e1_blueprints = [x for x in world.get_blueprint_library().filter('vehicle.*') if int(x.get_attribute('number_of_wheels')) == 4]
e1_blueprint = random.choice(e1_blueprints)
e1_blueprint.set_attribute('color', '255,0,0') # set e1 red
e1_actor = world.spawn_actor(e1_blueprint, e1_transform)
e1_actor.set_simulate_physics(True)

# spawn vehicle close to e2
e2_transform = carla.Transform(carla.Location(x=100.4, y=139.7, z=1.0))
e2_blueprints = [x for x in world.get_blueprint_library().filter('vehicle.*') if int(x.get_attribute('number_of_wheels')) == 4]
e2_blueprint = random.choice(e2_blueprints)
e2_blueprint.set_attribute('color', '0,255,0') # set e2 green
e2_actor = world.spawn_actor(e2_blueprint, e2_transform)
e2_actor.set_simulate_physics(True)
