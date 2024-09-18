import carla
import os
import threading
import matplotlib.pyplot as plt

def visualize_route_in_carla(world, route, life_time=1.0):
    for i in range(len(route) - 1):
        wp_start = route[i]
        wp_end = route[i + 1]
        # Draw a line between waypoints
        world.debug.draw_line(wp_start.transform.location, wp_end.transform.location,
                              thickness=0.1, color=carla.Color(0, 255, 0), life_time=life_time)
        # Draw waypoints
        world.debug.draw_point(wp_start.transform.location, size=0.1,
                               color=carla.Color(255, 0, 0), life_time=life_time)
    # Draw the last waypoint
    world.debug.draw_point(route[-1].transform.location, size=0.1,
                           color=carla.Color(255, 0, 0), life_time=life_time)


def calculate_overhead_camera_transform(route):
    x_coords = [wp.transform.location.x for wp in route]
    y_coords = [wp.transform.location.y for wp in route]
    z_coords = [wp.transform.location.z for wp in route]

    center_x = (max(x_coords) + min(x_coords)) / 2
    center_y = (max(y_coords) + min(y_coords)) / 2
    max_z = max(z_coords)

    # Elevate the camera sufficiently above the scene
    center_z = max_z + 100  # You may need to adjust this value

    location = carla.Location(x=center_x, y=center_y, z=center_z)
    rotation = carla.Rotation(pitch=-90)  # Pointing downward

    return location, rotation

def spawn_camera(world, location, rotation, image_width=1920, image_height=1080, fov=90):
    blueprint_library = world.get_blueprint_library()
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', str(image_width))
    camera_bp.set_attribute('image_size_y', str(image_height))
    camera_bp.set_attribute('fov', str(fov))

    camera_transform = carla.Transform(location, rotation)
    camera = world.spawn_actor(camera_bp, camera_transform)

    return camera


def capture_image(world, location, rotation, save_path):
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '800')
    camera_bp.set_attribute('image_size_y', '600')
    camera_bp.set_attribute('fov', '90')

    camera_transform = carla.Transform(location, rotation)
    camera = world.spawn_actor(camera_bp, camera_transform)

    image_saved_event = threading.Event()
    image_filename = os.path.join(save_path, 'overview.png')

    def save_image(image):
        if not image_saved_event.is_set():
            image.save_to_disk(image_filename)
            print(f"Image saved to {image_filename}")
            image_saved_event.set()

    camera.listen(save_image)

    try:
        # Wait for the image to be saved
        while not image_saved_event.is_set():
            if world.get_settings().synchronous_mode:
                world.tick()
            else:
                world.wait_for_tick()
    finally:
        if camera.is_alive:
            camera.stop()
            camera.destroy()





