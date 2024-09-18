import carla
import pygame
import random
import os
import argparse
import time
import queue

from matplotlib.pyplot import connect

from agents.navigation.basic_agent import BasicAgent
from agents.navigation.local_planner import RoadOption

# Import your custom modules (ensure they are correctly implemented)
from generate_route import generate_route_with_turns, spawn_obstacles_along_route
from utils.visualization import visualize_route_in_carla, calculate_overhead_camera_transform, capture_image
from utils.determine_actions import determine_actions

def save_actions(actions, save_path):
    actions_filename = os.path.join(save_path, 'actions.txt')
    with open(actions_filename, 'w') as f:
        for action in actions:
            f.write(action + '\n')

def drive_vehicle_along_route(world, vehicle, route, actions, index, camera_bp, save_path):
    agent = BasicAgent(vehicle)
    destination = route[-1].transform.location
    agent.set_destination(destination)
    plan = [(wp, RoadOption.LANEFOLLOW) for wp in route]
    agent._local_planner.set_global_plan(plan)

    # 设置车载相机
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

    # 创建保存图片的目录
    image_dir = os.path.join(save_path, 'images')
    os.makedirs(image_dir, exist_ok=True)

    # 创建图像队列
    image_queue = queue.Queue()
    camera.listen(image_queue.put)

    action_index = 0  # 当前 action 的索引

    try:
        while True:
            world.tick()

            # 获取最新的图像
            # while not image_queue.empty():
            image = image_queue.get()

            control = agent.run_step()
            vehicle.apply_control(control)

            vehicle_location = vehicle.get_location()

            if action_index < len(actions):
                # 获取当前 action 对应的 waypoint
                waypoint = route[action_index]
                distance = vehicle_location.distance(waypoint.transform.location)

                if distance < 1.0:  # 根据需要调整阈值
                    # 保存图片
                    image_filename = os.path.join(image_dir, f"action_{action_index}.png")
                    image.save_to_disk(image_filename)
                    print(f"Saved image for action {action_index}, distance: {distance:.2f}")
                    action_index += 1
                else:
                    # 检查车辆是否已经超过了 waypoint
                    if action_index + 1 < len(route):
                        next_waypoint = route[action_index + 1]
                        next_distance = vehicle_location.distance(next_waypoint.transform.location)
                        if next_distance < distance:
                            # 车辆已经经过了当前 waypoint，保存图片
                            image_filename = os.path.join(image_dir, f"action_{action_index}.png")
                            image.save_to_disk(image_filename)
                            print(f"Saved image for action {action_index} after passing waypoint")
                            action_index += 1
                    else:
                        # 最后一个 waypoint
                        image_filename = os.path.join(image_dir, f"action_{action_index}.png")
                        image.save_to_disk(image_filename)
                        print(f"Saved image for action {action_index} at end of route")
                        action_index += 1

            if agent.done() or action_index >= len(actions):
                print(f"Vehicle has reached the destination on route {index}.")
                break
    finally:
        # 清理
        if camera.is_alive:
            camera.stop()
            camera.destroy()
        if vehicle.is_alive:
            vehicle.destroy()


def spawn_vehicle_at_waypoint(world, waypoint, vehicle_bp):
    transform = waypoint.transform
    vehicle = world.try_spawn_actor(vehicle_bp, transform)
    return vehicle


def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='CARLA Control Script')
    parser.add_argument('-host', type=str, default='localhost', help='Host to connect to CARLA server')
    parser.add_argument('-port', type=int, default=2000, help='Port to connect to CARLA server')
    parser.add_argument('-save_path', type=str, default='../output115', help='Path to save the images')
    parser.add_argument('-world', type=str, default='Town01', help='World to load')
    # Weather parameters can be added here if needed

    args = parser.parse_args()

    pygame.init()
    pygame.font.init()

    display = pygame.display.set_mode((1200, 800), pygame.HWSURFACE | pygame.DOUBLEBUF)
    pygame.display.set_caption("CARLA Simulation")

    clock = pygame.time.Clock()


    # Connect to CARLA server
    # client = carla.Client(args.host, args.port)
    # client.set_timeout(20.0)



    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.load_world(args.world)

    # Initialize variables
    min_length_1_turn = 25
    max_length_1_turn = 34

    min_length_2_turns = 35
    max_length_2_turns = 49

    route_1_turn_index = 0
    route_2_turns_index = 0
    routes_1_turn = []
    routes_2_turns = []

    routes_1_save_path = os.path.join(args.save_path, args.world, 'route_1_turn')
    os.makedirs(routes_1_save_path, exist_ok=True)

    routes_2_save_path = os.path.join(args.save_path, args.world, 'route_2_turn')
    os.makedirs(routes_2_save_path, exist_ok=True)


    try:
        carla_map = world.get_map()
        waypoints = carla_map.generate_waypoints(distance=5.0)

        # Save the original settings
        original_settings = world.get_settings()

        # Enable synchronous mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # Adjust as needed
        world.apply_settings(settings)

        # Sample waypoints
        sampled_waypoints = random.sample(waypoints[:30], 20)  # Reduce the number to reduce load

        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.*')[0]

        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '800')
        camera_bp.set_attribute('image_size_y', '600')
        camera_bp.set_attribute('fov', '90')


        # Generate routes
        for sampled_waypoint in sampled_waypoints:
            # Generate a route with 1 turn
            route_1_turn = generate_route_with_turns(
                sampled_waypoint, min_length_1_turn, max_length_1_turn, num_turns=1, angle_threshold=10.0
            )

            if route_1_turn:
                save_1_path = os.path.join(routes_1_save_path, f'route_{route_1_turn_index}')
                os.makedirs(save_1_path, exist_ok=True)

                # Visualize the route
                visualize_route_in_carla(world, route_1_turn, life_time=1.0)
                world.tick()  # Advance simulation to ensure drawings appear

                # Capture image
                location, rotation = calculate_overhead_camera_transform(route_1_turn)
                try:
                    capture_image(world, location, rotation, save_1_path)
                except Exception as e:
                    print(f"Error while capturing image for route {route_1_turn_index}: {e}")

                # Advance simulation to clear debug drawings
                ticks_to_clear = int(1.0 / settings.fixed_delta_seconds)
                for _ in range(ticks_to_clear):
                    world.tick()

                # Append route data
                routes_1_turn.append({'index': route_1_turn_index, 'type': '1_turn', 'route': route_1_turn})
                route_1_turn_index += 1


                # save_path = os.path.join(save_1_path, f'route_{route_1_turn_index}')

                vehicle = spawn_vehicle_at_waypoint(world, route_1_turn[0], vehicle_bp)
                if vehicle:
                    try:
                        actions = determine_actions(route_1_turn)
                        save_actions(actions, save_1_path)

                        drive_vehicle_along_route(world, vehicle, route_1_turn, actions, route_1_turn_index, camera_bp, save_1_path)
                    except Exception as e:
                        print(f"Error while driving along route {route_1_turn_index}: {e}")
                    finally:
                        if vehicle.is_alive:
                            vehicle.destroy()
                else:
                    print(f"Failed to spawn vehicle for route {route_1_turn_index}")


            # Generate a route with 2 turns
            route_2_turns = generate_route_with_turns(
                sampled_waypoint, min_length_2_turns, max_length_2_turns, num_turns=2, angle_threshold=10.0
            )
            if route_2_turns:
                save_2_path = os.path.join(routes_2_save_path, f'route_{route_2_turns_index}')
                os.makedirs(save_2_path, exist_ok=True)

                # Visualize the route
                visualize_route_in_carla(world, route_2_turns, life_time=1.0)
                world.tick()  # Advance simulation to ensure drawings appear

                # Capture image
                location, rotation = calculate_overhead_camera_transform(route_2_turns)
                try:
                    capture_image(world, location, rotation, save_2_path)
                except Exception as e:
                    print(f"Error while capturing image for route {route_2_turns_index}: {e}")

                # Advance simulation to clear debug drawings
                ticks_to_clear = int(1.0 / settings.fixed_delta_seconds)
                for _ in range(ticks_to_clear):
                    world.tick()

                # Append route data
                routes_2_turns.append({'index': route_2_turns_index, 'type': '2_turns', 'route': route_2_turns})
                route_2_turns_index += 1

                vehicle = spawn_vehicle_at_waypoint(world, route_2_turns[0], vehicle_bp)
                if vehicle:
                    try:
                        actions = determine_actions(routes_2_turns)
                        save_actions(actions, save_2_path)

                        drive_vehicle_along_route(world, vehicle, route_2_turns, actions, route_2_turns_index, camera_bp,
                                                  save_2_path)
                    except Exception as e:
                        print(f"Error while driving along route {route_2_turns_index}: {e}")
                    finally:
                        if vehicle.is_alive:
                            vehicle.destroy()
                else:
                    print(f"Failed to spawn vehicle for route {route_2_turns_index}")

        print('ROUTE 1 TURN:', len(routes_1_turn))
        print('ROUTE 2 TURNS:', len(routes_2_turns))



        # # Drive along the 1-turn routes
        # for route_data in routes_1_turn:
        #     index = route_data['index']
        #     route = route_data['route']
        #     save_path = os.path.join(routes_1_save_path, f'route_{index}')
        #
        #     vehicle = spawn_vehicle_at_waypoint(world, route[0], vehicle_bp)
        #     if vehicle:
        #         try:
        #             actions = determine_actions(route)
        #             save_actions(actions, save_path)
        #
        #             drive_vehicle_along_route(world, vehicle, route, actions, index, camera_bp, save_path)
        #         except Exception as e:
        #             print(f"Error while driving along route {index}: {e}")
        #         finally:
        #             if vehicle.is_alive:
        #                 vehicle.destroy()
        #     else:
        #         print(f"Failed to spawn vehicle for route {index}")

    except Exception as e:
        print(f"An error occurred: {e}")


    finally:
        # Reset settings
        try:
            if world is not None:
                world.apply_settings(original_settings)
                # Clean up any remaining actors
                actors = world.get_actors()

                for actor in actors:
                    if actor.is_alive:
                        actor.destroy()
        except Exception as e:
            print(f"Error during cleanup: {e}")
            pass

if __name__ == '__main__':

    try:

        main()

    except KeyboardInterrupt:

        print(' - Exited by user.')