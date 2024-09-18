import carla
import random
import os
import argparse
import queue
import threading
import math
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

def drive_vehicle_along_route(world, vehicle, route, index, camera_bp, save_path):
    agent = BasicAgent(vehicle)
    destination = route[-1].transform.location
    agent.set_destination(destination)
    plan = [(wp, RoadOption.LANEFOLLOW) for wp in route]
    agent._local_planner.set_global_plan(plan)

    # Set up the camera sensor
    camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

    # Directory to save images
    image_dir = os.path.join(save_path, 'images')
    os.makedirs(image_dir, exist_ok=True)

    # Create a queue to store images
    image_queue = queue.Queue()
    camera.listen(image_queue.put)

    waypoint_index = 0

    try:
        while True:
            world.tick()

            # # Retrieve the latest image
            # while not image_queue.empty():
            #     image = image_queue.get()
            image = image_queue.get()
            control = agent.run_step()
            vehicle.apply_control(control)

            vehicle_location = vehicle.get_location()

            if waypoint_index < len(route):
                waypoint = route[waypoint_index]
                distance = vehicle_location.distance(waypoint.transform.location)
                if distance < 1.0:  # Adjust threshold as needed
                    image_filename = os.path.join(image_dir, f"waypoint_{waypoint_index}.png")
                    image.save_to_disk(image_filename)
                    print(f"Saved image at waypoint {waypoint_index}, distance: {distance:.2f}")
                    waypoint_index += 1

            if agent.done() or waypoint_index >= len(route):
                print(f"Vehicle has reached the destination on route {index}.")
                break
    except Exception as e:
        print(f"Error in drive_vehicle_along_route for route {index}: {e}")
    finally:
        # Cleanup
        if camera.is_alive:
            camera.stop()
            camera.destroy()
        if vehicle.is_alive:
            vehicle.destroy()

def spawn_vehicle_at_waypoint(world, waypoint, vehicle_bp):
    transform = waypoint.transform
    vehicle = world.try_spawn_actor(vehicle_bp, transform)
    return vehicle

# def capture_image(world, location, rotation, save_path):
#     camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
#     camera_bp.set_attribute('image_size_x', '800')
#     camera_bp.set_attribute('image_size_y', '600')
#     camera_bp.set_attribute('fov', '90')
#
#     camera_transform = carla.Transform(location, rotation)
#     camera = world.spawn_actor(camera_bp, camera_transform)
#
#     image_saved_event = threading.Event()
#     image_filename = os.path.join(save_path, 'overview.png')
#
#     def save_image(image):
#         if not image_saved_event.is_set():
#             image.save_to_disk(image_filename)
#             print(f"Image saved to {image_filename}")
#             image_saved_event.set()
#
#     camera.listen(save_image)
#
#     try:
#         # Wait for the image to be saved
#         while not image_saved_event.is_set():
#             if world.get_synchronous_mode():
#                 world.tick()
#             else:
#                 world.wait_for_tick()
#     finally:
#         if camera.is_alive:
#             camera.stop()
#             camera.destroy()

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description='CARLA Control Script')
    parser.add_argument('-host', type=str, default='localhost', help='Host to connect to CARLA server')
    parser.add_argument('-port', type=int, default=2000, help='Port to connect to CARLA server')
    parser.add_argument('-save_path', type=str, default='../output111', help='Path to save the images')
    parser.add_argument('-world', type=str, default='Town01', help='World to load')
    # Weather parameters can be added here if needed

    args = parser.parse_args()

    # Connect to CARLA server
    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)

    try:
        # Load world
        world = client.load_world(args.world)
        carla_map = world.get_map()
        waypoints = carla_map.generate_waypoints(distance=5.0)

        # Save the original settings
        original_settings = world.get_settings()

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

        # Enable synchronous mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05  # Adjust as needed
        world.apply_settings(settings)

        # Sample waypoints
        sampled_waypoints = random.sample(waypoints, 10)  # Reduce the number to reduce load

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

        print('ROUTE 1 TURN:', len(routes_1_turn))
        print('ROUTE 2 TURNS:', len(routes_2_turns))

        blueprint_library = world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.*')[0]

        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '800')
        camera_bp.set_attribute('image_size_y', '600')
        camera_bp.set_attribute('fov', '90')

        # Drive along the 1-turn routes
        for route_data in routes_1_turn:
            index = route_data['index']
            route = route_data['route']
            save_path = os.path.join(routes_1_save_path, f'route_{index}')

            vehicle = spawn_vehicle_at_waypoint(world, route[0], vehicle_bp)
            if vehicle:
                try:
                    drive_vehicle_along_route(world, vehicle, route, index, camera_bp, save_path)

                    actions = determine_actions(route)
                    save_actions(actions, save_path)
                except Exception as e:
                    print(f"Error while driving along route {index}: {e}")
                finally:
                    if vehicle.is_alive:
                        vehicle.destroy()
            else:
                print(f"Failed to spawn vehicle for route {index}")

    except Exception as e:
        print(f"An error occurred in the main function: {e}")

    finally:
        # Reset settings
        if world:
            world.apply_settings(original_settings)

            # Clean up any remaining actors
            actors = world.get_actors()
            for actor in actors:
                if actor.is_alive:
                    actor.destroy()
        else:
            print("World is not available. CARLA server may have been closed.")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print(' - Exited by user.')
