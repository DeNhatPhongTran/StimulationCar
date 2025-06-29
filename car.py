import math
import numpy as np
import obj as obj_lib
import sensor as sensor_lib


class ObjCar(obj_lib.ObjImageMove):
    """car class"""

    def __init__(self, pygame, screen, image_file, intial_direction):
        super().__init__(pygame, screen, image_file, intial_direction)
        self.speed_prev = None
        self.max_speed = 50
        self.acceleration = 5
        self.acceleration_max = 20
        self.acc_brake = 5
        self.speed = 0
        self.sensors = self.init_sensors()


    def init_sensors(self):
        sensors = []
        sensors.append(sensor_lib.SensorSimulator(self.pygame, self.screen))
        return sensors

    def draw_heading(self, heading):
        def degree2rad(heading):
            return heading * math.pi / 180

        ## draw_heading()
        length = 75
        start_point = self.gnav('center')
        theta = degree2rad(heading)
        end_x = start_point[0] + round(length * np.cos(theta))
        end_y = start_point[1] + -round(length * np.sin(theta))
        self.pygame.draw.lines(self.screen, (0, 255, 0), False, [start_point, (end_x, end_y)], 1)

    def set_delta_time(self, delta_time):
        self.delta_time = delta_time

    def update(self, map):
        status = self.get_status(map)
        for sensor in self.sensors:
            new_process = sensor.collect(status)
            if new_process:
                return new_process
        return None
    
    def update_position(self):
        dx = self.speed * self.delta_time * math.cos(self.heading)
        dy = -self.speed * self.delta_time * math.sin(self.heading)
        self.x += dx
        self.y += dy

    def set_speed(self):
        if self.speed > self.max_speed:
            self.speed -= self.acceleration 
            if self.speed < self.max_speed:
                self.speed = self.max_speed 
        elif self.speed < self.max_speed:
            self.speed += self.acceleration
            if self.speed > self.max_speed:
                self.speed = self.max_speed 

    
    def set_max_speed(self, max_speed):
        self.max_speed = max_speed

    def get_activate_distance(self):
        v = self.speed
        v0 = 5
        a = self.acc_brake
        distance_meter = abs((v ** 2 - v0 ** 2) / (2 * a))
        pixel_per_meter = 153 / 1000
        distance_pixel = distance_meter * pixel_per_meter
        return int(distance_pixel)
        
    def draw_centered_square(self, size = None, color=(0, 0, 255)):
        if size is None:
            size = self.get_activate_distance() + 30
        center = self.gnav('center')
        rect = self.pygame.Rect(
            center[0] - size // 2,
            center[1] - size // 2,
            size,
            size
        )
        self.pygame.draw.rect(self.screen, color, rect, 2) 


    def update_warning_collision_buffer(self):
        size = int(self.get_activate_distance() + 40)
        center = self.gnav('center')
        left = center[0] - size // 2
        top = center[1] - size // 2
        self.warning_collision_buffer = self.pygame.Rect(left, top, size, size)

   


    def slow_down(self):
        self.acceleration = 0
        self.speed_prev = self.speed
        if self.speed > 2:
            self.speed -= self.acc_brake
            if self.speed < 2:
                self.speed = 2

    def restore_speed(self):
        self.speed = self.speed_prev

    def reset(self):
        for sensor in self.sensors:
            sensor.reset()

    def get_status(self, map):
        # ping onboard systems for current status of car
        # status could be acquired from Inertial Measurement Unit (IMU), GPS receiver, spedometer, etc..
        status = {}
        status['car'] = self
        status['map'] = map
        status['location'] = self.get_status_location(status)
        return status

    def get_status_location(self, status):
        location = {}
        road = status['map'].get_road_obj(status['car'])
        location['road'] = road
        location['lane'] = road.get_lane(self)
        return location
