import carla
import random
import math
import numpy
import argparse
import sys
sys.path.append('/home/yanjun/CARLA/PythonAPI/carla')
# from agents.navigation.basic_agent import BasicAgent

from agents.navigation.global_route_planner import GlobalRoutePlanner

def find_random_destination(origin, min_distance, max_distance, map):
    while True:
        # ランダムな方向と距離を生成
        angle = random.uniform(0, 2 * math.pi)
        distance = random.uniform(min_distance, max_distance)
        # 終点の座標を計算
        destination = carla.Location(
            x=origin.x + distance * math.cos(angle),
            y=origin.y + distance * math.sin(angle)
        )
        # マップ上の道路上かどうかを確認
        if map.get_waypoint(destination, project_to_road=True, lane_type=carla.LaneType.Driving):
            return destination

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


def main():
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

    sensors = []
    action_list = []
    vehicle_actions = []
    landmarks = []

    try:
        client = carla.Client('localhost', 2001)
        client.set_timeout(2.0)
        world = client.load_world(args.world)
        map = world.get_map()

        debug = world.debug

        # ランダムな起点を選択
        spawn_points = map.get_spawn_points()
        start_location = random.choice(spawn_points)

        # 終点を計算
        end_location = find_random_destination(start_location.location, 400, 500, map)

        debug.draw_point(start_location.location + carla.Location(z=0.5), 0.1, carla.Color(r=255, g=0, b=0,a =0), 120.0)
        debug.draw_point(end_location + carla.Location(z=0.5), 0.1, carla.Color(r=0, g=255, b=0, a=0), 120.0)

        grp = GlobalRoutePlanner(map, 2.0)
        route = grp.trace_route(start_location.location, end_location)


        # 車両を生成
        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
        vehicle = world.spawn_actor(vehicle_bp, start_location)

        vehicle.set_route(route)

        # ナビゲーションを開始
        # ここでは、単純化のために、終点を直接設定していますが、実際にはルート計算や自動運転のロジックが必要です
        # vehicle.set_autopilot(True)  # 簡単なデモのためにオートパイロットを有効化

        print(f"Driving from {start_location.location} to {end_location}")

        # カメラセンサーを設定
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '800')
        camera_bp.set_attribute('image_size_y', '600')
        camera_bp.set_attribute('fov', '90')
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

        def image_callback(image):
            action = get_vehicle_action(vehicle)
            vehicle_actions.append(action)

            ego_vehicle_location = vehicle.get_location()
            current_waypoint = map.get_waypoint(ego_vehicle_location)
            landmark = current_waypoint.get_landmarks(2.0, True)
            landmarks.append(landmark)

            # image.save_to_disk(f'{save_path}/scene_{image.frame}.png')
            # print(f'Saved image at frame {image.frame} with action {action} at landmarks {landmark}')

        camera.listen(image_callback)

        spectator = world.get_spectator()
        spectator.set_transform(camera.get_transform())

        camera.stop()
    finally:
        print('Closing CARLA client')
        client.apply_batch([carla.command.DestroyActor(x) for x in action_list])


if __name__ == '__main__':
    main()
