import carla
import random
import time
import glob
import os
import sys
import argparse
import math
from PIL import ImageGrab
from agents.navigation.basic_agent import BasicAgent
from agents.navigation.behavior_agent import BehaviorAgent
from traffic_event import TrafficEvent

def set_weather(world):
    weather = carla.WeatherParameters(cloudiness=0,
                                      precipitation=0,
                                      fog_density=0,
                                      sun_azimuth_angle=10)

    world.set_weather(weather)

def get_vehicle_action(vehicle):
    velocity = vehicle.get_velocity()
    speed = math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
    control = vehicle.get_control()
    if speed < 0.1:
        return 'stop'
    if control.steer < -0.2:
        return 'left'
    if control.steer > 0.2:
        return 'right'
    else:
        return 'forward'

def calculate_distance(loc1, loc2):
    dx = loc1.location.x - loc2.location.x
    dy = loc1.location.y - loc2.location.y
    dz = loc1.location.z - loc2.location.z
    distance = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

    return distance

def main():
    # コマンドライン引数をパースするための設定
    parser = argparse.ArgumentParser(description='CARLA Control Script')
    parser.add_argument('-host', type=str, default='localhost', help='Host to connect to CARLA server')
    parser.add_argument('-port', type=int, default=2001, help='Port to connect to CARLA server')
    parser.add_argument('-save_path', type=str, default='../output0508', help='Path to save the images')
    parser.add_argument('-world', type=str, default='Town01', help='World to load')
    ## weather
    parser.add_argument('--cloudiness', type=float, default=10.0, help='')
    parser.add_argument('--precipitation', type=float, default=10.0, help='')
    parser.add_argument('--precipitation_deposits', type=float, default=10.0, help='')
    parser.add_argument('--wind_intensity', type=float, default=10.0, help='')
    parser.add_argument('--sun_azimuth_angle', type=float, default=10.0, help='')
    parser.add_argument('--sun_altitude_angle', type=float, default=10.0, help='')

    args = parser.parse_args()

    action_list = []
    vehicle_actions = []
    landmarks = []
    try:
        # CARLAサーバーに接続
        client = carla.Client(args.host, args.port)
        client.set_timeout(2.0)

        # worlds
        world = client.load_world(args.world)
        map = world.get_map()
        # weather
        # set_weather(world)

        spawn_points = world.get_map().get_spawn_points()
        location_spawn_point = [point.location for point in spawn_points]

        # waypoints = map.generate_waypoints(2.0)

        for i, spawn_point in enumerate(spawn_points[:3]):
            print('===========================Spawning point {}====================='.format(i))
            save_path = f'{args.save_path}/{args.world}/spawn_{i}'
            if not os.path.exists(save_path):
                os.makedirs(save_path)

            start_location = spawn_point
            far_locations = [point for point in spawn_points if calculate_distance(start_location, point) > 100]
            if len(far_locations) == 0:
                continue
            end_location = random.choice(far_locations)
            print('start_location', start_location)
            print('end_location', end_location)

            blueprint_library = world.get_blueprint_library()
            vehicle_bp = blueprint_library.filter('vehicle.*')[0]  # audi a2
            vehicle = world.spawn_actor(vehicle_bp, start_location)

            agent = BehaviorAgent(vehicle, behavior='normal')
            agent.set_destination(end_location.location)

            # TODO: add another vehicle

            route = agent.get_local_planner().get_plan()
            junctions = []
            for waypoint, _ in route:
                map_wp = map.get_waypoint(waypoint.transform.location)
                if map_wp.is_junction:
                    junctions.append(map_wp)
            # route_waypoints = [map.get_waypoint(wp) for wp in route]

            # traffic_event = TrafficEvent(world, location, action, weather_params)
            # junctions = [wp for wp in route_waypoints if wp.is_junction]
            if len(junctions) != 0:
                junction = random.choice(junctions)
                traffic_event = TrafficEvent(world, junction)
                traffic_event.add_vehicle_at_junction()


            camera_bp = blueprint_library.find('sensor.camera.rgb')
            camera_bp.set_attribute('image_size_x', '800')
            camera_bp.set_attribute('image_size_y', '600')
            camera_bp.set_attribute('fov', '90')
            camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
            camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)


            route_points = []
            vehicle_actions = []
            frame_number = 0

            spectator = world.get_spectator()
            def image_callback(image):
                nonlocal frame_number
                action = get_vehicle_action(vehicle)
                vehicle_actions.append(action)

                ego_vehicle_location = vehicle.get_location()
                current_waypoint = map.get_waypoint(ego_vehicle_location)
                landmark = current_waypoint.get_landmarks(2.0, True)
                landmarks.append(landmark)

                frame_number += 1
                image.save_to_disk(f'{save_path}/scene_{frame_number}.png')
                print(f'Saved image at frame {frame_number} with action {action} at landmarks {landmark}')

            camera.listen(image_callback)

            while not agent.done():
                world.tick()
                vehicle.apply_control(agent.run_step())
                location = vehicle.get_location()
                route_points.append(location)
                action_list.append(vehicle)

                # sensors.append(camera)
                spectator.set_transform(camera.get_transform())

                if location in location_spawn_point:
                    image_callback(camera)

            with open(f'{save_path}/action.txt', 'w') as f, open(f'{save_path}/landmarks.txt', 'w') as f1:
                for action in vehicle_actions:
                    f.write(f'{action}\n')
                for landmark in landmarks:
                    f1.write(f'{landmark}\n')

            # camera.stop()

    finally:
        print('Closing CARLA client')
        client.apply_batch([carla.command.DestroyActor(x) for x in action_list])
        #




if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')