import random

import carla


class TrafficEvent:
    def __init__(self, world, location):
        self.world = world
        # self.event_type = event_type
        self.location = location
        # self.action = action
        # self.weather_params = weather_params

    def add_pedestrian(self):
        blueprint_library = self.world.get_blueprint_library()
        pedestrain_bps = blueprint_library.filter("walker.*")
        pedestrain_bp = random.choice(pedestrain_bps)
        pedestrain = self.world.spawn_actor(pedestrain_bp, self.location)
        return pedestrain

    def add_vehicle_at_junction(self):
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bps = blueprint_library.filter("vehicle.*")
        vehicle_bp = random.choice([vehicle_bp for vehicle_bp in vehicle_bps if vehicle_bp.id != 'vehicle.audi.a2'])

        # spawn_transform = carla.Transform(self.location)

        for _ in range(10):
            try:
                vehicle = self.world.spawn_actor(vehicle_bp, self.location.transform)
            except RuntimeError as e:
                print(f"Spawn failed due to collision: {e}")
                self.location.transform.location.x += random.uniform(-2.0, 2.0)
                self.location.transform.location.y += random.uniform(-2.0, 2.0)
        print("Failed to spaen vehicle at junction after multiple attempts.")
        return None

    def add_speical_car(self):
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter('vehicle.ford.ambulance')[0]
        vehicle = self.world.spawn_actor(vehicle_bp, self.location)
        return vehicle

    # def change_weather(self):
    #     self.world.set_weather(self.weather_params)


    # def stop_for_vehicle(self, vehicle, traffic_manager):
    #     traffic_manager.vehicle_percentage_speed_difference(vehicle, -100)
    #     print("Vehicle is stopped for waiting")
    #     while not is_crossing_car_passed():
    #         self.world.tick()

    # def stop_for_pedestrain(self, vehicle, traffic_manager):
    #     traffic_manager.vehicle_percentage_speed_difference(vehicle, -100)
    #     print("Vehicle is stopped for waiting")
    #     while not is_pedestrain_passed():
    #         self.world.tick()
