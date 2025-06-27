import obj as obj_lib
import road_artifact
import drive as drive_lib
import utilities as u

class Sensor(obj_lib.Obj):
    """
    parent object class for car sensors

    returns instruction
        driving instruction - (heading, speed)
        no driving instruction (no new process or process has completed) - None
        arrived at destination - 'arrived'
    """

    def __init__(self, pygame, screen):
        super().__init__(pygame, screen)
        self.classifiers = []
        self.init_classifiers()

    def add_classifier(self, object):
        self.classifiers.append(object(self.pygame, self.screen))

    def init_classifiers(self):
        pass

    def collect(self, status):
        raw_data = self.retrieve(status)
        for c in self.classifiers:
            new_process = c.evaluate(status, raw_data)
            if new_process:
                return new_process
        return None

    def retrieve(self, status):
        pass

    def reset(self):
        for c in self.classifiers:
            c.reset()


class SensorSimulator(Sensor):
    def __init__(self, pygame, screen):
        super().__init__(pygame, screen)

    def init_classifiers(self):
        self.add_classifier(ClassiferSimulatorStationaryDestination)
        self.add_classifier(ClassiferSimulatorStationarySignSpeed15)
        self.add_classifier(ClassiferSimulatorStationarySignSpeed25)
        self.add_classifier(ClassiferSimulatorStationarySignSpeed45)
        self.add_classifier(ClassiferSimulatorStationarySignSpeed55)
        self.add_classifier(ClassiferSimulatorStationarySignSpeed65)
        self.add_classifier(ClassiferSimulatorStationarySignStop)
        self.add_classifier(ClassiferSimulatorStationarySignTrafficLight)
        self.add_classifier(ClassiferSimulatorMoveVehicle)
        self.add_classifier(ClassiferSimulatorMovePedestrian)

    def retrieve(self, status):
        # return artifacts that the car has not passed
        car = status['car']
        road = status['location']['road']
        artifacts = status['location']['road'].artifacts
        car_bottom = car.gnav('midbottom')

        visible_artifacts = []
        for artifact in artifacts:
            artifact_top = artifact.gnav('midtop')
            if road.dir_val_exceeds(artifact_top, car_bottom):
                visible_artifacts.append(artifact)
        return visible_artifacts


class Classifier(obj_lib.Obj):
    """parent object class for sensor classifiers"""

    def __init__(self, pygame, screen, activate_distance):
        super().__init__(pygame, screen)
        self.status = {}
        self.activate_distance = activate_distance
        self.activate_distance_buffer = 0

    def evaluate(self, status, raw_data):
        feature = self.extract(status, raw_data)
        if not feature:
            return None
        if not self.activate(status, feature):
            return None
        return self.get_process(status, feature)

    def extract(self, status, raw_data):
        # if raw_data handled by classifier, then return structured data from raw data
        pass

    def status_is_inactive(self, id):
        # if status for feature with id has not been set, set it to inactive
        # otherwise, return true if feature with id's status is inactive
        if id not in self.status:
            self.status[id] = 'inactive'
            return True
        else:
            return self.status[id] == 'inactive'

    def status_set_active(self, feature):
        # set the status for feature active
        id = feature['id']
        self.status[id] = 'active'

    def status_set_inactive(self, feature):
        # set the status for feature active
        id = feature['id']
        self.status[id] = 'inactive'

    def status_set_complete(self, feature):
        # set the status for feature to complete
        id = feature['id']
        self.status[id] = 'complete'

    def activate(self, status, feature):
        # return true if process for feature should be activated, false otherwise
        pass

    def get_process(self, status, feature):
        self.status_set_active(feature)
        data = self.get_process_data(status, feature)
        return (data, self.process_function)

    def get_process_data(self, status, feature):
        return {'status': status, 'feature': feature}

    def process_function(self, data):
        pass

    def send_instruction(self, car, heading, speed, text, interval= 1000):
        if not hasattr(self, 'first_time'):
            self.first_time = self.pygame.time.get_ticks()
            car.draw_outline(text)

        now = self.pygame.time.get_ticks()
        if now - self.first_time > interval:    
            car.draw_outline(text)
            self.first_time = self.pygame.time.get_ticks()
        return car.make_instruction(heading, speed)

    def reset(self):
        self.status = {}


class ClassifierSimulator(Classifier):
    def __init__(self, pygame, screen, artifact_class, activate_distance, activate_pos):
        super().__init__(pygame, screen, activate_distance)
        self.activate_distance_buffer = 5  # length of car
        self.artifact_class = artifact_class
        self.activate_pos = activate_pos
        self.status = {}

    def get_artifact_id(self, artifact):
        road_id = artifact.road.id
        artifact_id = artifact.id
        return (road_id, artifact_id)

    def extract(self, status, raw_data):
        for artifact in raw_data:
            if isinstance(artifact, self.artifact_class):
                if self.status_is_inactive(self.get_artifact_id(artifact)):
                    return self.extract_data(status, artifact)
        return None

    def extract_data(self, status, artifact):
        feature = {'artifact': artifact, 'id': self.get_artifact_id(artifact)}

        car = status['car']
        road = status['location']['road']

        # distance - difference between artifact position and bottom of the car
        if self.activate_pos:
            pos_artifact = artifact.gnav(self.activate_pos)
        else:
            # segment position
            location_road = artifact.pos_parms['length_attribute_road']
            pos_artifact = road.gnav(location_road)
        pos_car = car.gnav('top')

        feature['distance'] = (pos_artifact - pos_car) * road.graph_dir_length
        feature['heading'] = u.heading(car.center, artifact.center)

        # same_lane
        # * none - artifact is not in a lane
        # * True - artifact is in the same lane as the car
        # * False - artifact is in a lane, but not the car's lane
        artifact_lane_id = artifact.pos_width
        if type(artifact_lane_id) is int:
            feature['same_lane'] = artifact_lane_id == status['location']['lane'].lane_id
        else:
            feature['same_lane'] = None

        return feature

    def activate(self, status, feature):
        same_lane = feature['same_lane']
        if same_lane is False:
            return False
        if 'activate_distance' not in feature:
            feature['activate_distance'] = status['car'].get_activate_distance()
        activate_distance = feature['activate_distance']
        return feature['distance'] <= (activate_distance + self.activate_distance_buffer)

    def process_complete(self, feature):
        pass

    def in_warning_collision_buffer(self, car, artifact):
        return car.warning_collision_buffer and not car.warning_collision_buffer.is_clear([artifact])
    
    def in_collision_buffer(self, car, artifact):
        return car.collision_buffer and not car.collision_buffer.is_clear([artifact])


class ClassiferSimulatorStationary(ClassifierSimulator):
    def __init__(self, pygame, screen, artifact_class, activate_distance, activate_pos):
        super().__init__(pygame, screen, artifact_class, activate_distance, activate_pos)

    def process_complete(self, feature):
        self.status_set_complete(feature)

class ClassiferSimulatorStationaryDestination(ClassiferSimulatorStationary):
    def __init__(self, pygame, screen):
        super().__init__(pygame, screen, road_artifact.ObjRoadArtifactStationaryDestination, 0, None)

    def get_process_data(self, status, feature):
        data = super().get_process_data(status, feature)
        wait_time = 3  # seconds
        data['complete'] = self.pygame.time.get_ticks() + (wait_time * 1000)
        return data

    def process_function(self, data):
        car = data['status']['car']
        ticks = self.pygame.time.get_ticks()
        if ticks < data['complete']:
            return self.send_instruction(car, None, 0, 'Waiting at destination')
        else:
            return 'arrived'


class ClassiferSimulatorStationarySignSpeed(ClassiferSimulatorStationary):
    def __init__(self, pygame, screen, artifact_class, activate_distance, activate_pos, max_speed):
        self.max_speed = max_speed
        super().__init__(pygame, screen, artifact_class, activate_distance, activate_pos)

    def process_function(self, data):
        car = data['status']['car']

        if car.max_speed != self.max_speed:
            car.max_speed = self.max_speed  # allow temporary speed changes to be reset
            return self.send_instruction(car, None, self.max_speed, f'Setting max speed to: {self.max_speed}', 1000)
        else:
            feature = data['feature']
            self.process_complete(feature)

class ClassiferSimulatorStationarySignSpeed15(ClassiferSimulatorStationarySignSpeed):
    def __init__(self, pygame, screen):
        super().__init__(pygame, screen, road_artifact.ObjRoadArtifactStationarySignSpeed15, 0, None, 15)


class ClassiferSimulatorStationarySignSpeed25(ClassiferSimulatorStationarySignSpeed):
    def __init__(self, pygame, screen):
        super().__init__(pygame, screen, road_artifact.ObjRoadArtifactStationarySignSpeed25, 0, None, 25)


class ClassiferSimulatorStationarySignSpeed45(ClassiferSimulatorStationarySignSpeed):
    def __init__(self, pygame, screen):
        super().__init__(pygame, screen, road_artifact.ObjRoadArtifactStationarySignSpeed45, 0, None, 45)


class ClassiferSimulatorStationarySignSpeed55(ClassiferSimulatorStationarySignSpeed):
    def __init__(self, pygame, screen):
        super().__init__(pygame, screen, road_artifact.ObjRoadArtifactStationarySignSpeed55, 0, None, 55)


class ClassiferSimulatorStationarySignSpeed65(ClassiferSimulatorStationarySignSpeed):
    def __init__(self, pygame, screen):
        super().__init__(pygame, screen, road_artifact.ObjRoadArtifactStationarySignSpeed65, 0, None, 65)


class ClassiferSimulatorStationarySignStop(ClassiferSimulatorStationary):
    def __init__(self, pygame, screen):
        super().__init__(pygame, screen, road_artifact.ObjRoadArtifactStationarySignStop, 0, None)

    def get_process_data(self, status, feature):
        data = super().get_process_data(status, feature)
        return data

    def process_function(self, data):
        car = data['status']['car']
        feature = data['feature']
        artifact = feature['artifact']
        road = data['status']['location']['road']

        pos_artifact = artifact.gnav('top') 
        pos_car = car.gnav('top')
        distance = (pos_artifact - pos_car) * road.graph_dir_length

        ticks = self.pygame.time.get_ticks()
        if distance > 5:
            car.slow_down()
            return self.send_instruction(car, None, car.speed, 'Slowing down for stop sign', 1000)
        elif distance <= 5:
            car.speed = 0
            if 'complete' not in data:
                wait_time = 3  # seconds
                data['complete'] = ticks + (wait_time * 1000)
            if ticks < data['complete']:
                return self.send_instruction(car, None, car.speed, 'Waiting at stop sign', 1000)
            else:
                self.process_complete(data['feature'])
                car.acceleration = 15
                car.set_speed()


class ClassiferSimulatorStationarySignTrafficLight(ClassiferSimulatorStationary):
    def __init__(self, pygame, screen):
        super().__init__(pygame, screen, road_artifact.ObjRoadArtifactStationarySignTrafficLight, 0, None)

    def process_function(self, data):
        car = data['status']['car']
        feature = data['feature']
        artifact = feature['artifact']
        road = data['status']['location']['road']

        pos_artifact = artifact.gnav('top') 
        pos_car = car.gnav('top')
        distance = (pos_artifact - pos_car) * road.graph_dir_length

        ticks = self.pygame.time.get_ticks()
        if distance > 30:
            car.slow_down()
            return self.send_instruction(car, None, car.speed, 'Slowing down for traffic light',1000)
        elif distance <= 30:
            car.speed = 0
            if artifact.red:
                return self.send_instruction(car, None, 0, 'Waiting at red traffic light',1000)
            else:
                car.acceleration = 15
                car.set_speed()
                self.process_complete(feature)



class ClassifierSimulatorMove(ClassifierSimulator):
    def __init__(self, pygame, screen, artifact_class, activate_distance, activate_pos):
        super().__init__(pygame, screen, artifact_class, activate_distance, activate_pos)

    def process_complete(self, feature):
        self.status_set_inactive(feature)

    def status_set_inactive(self, data):
        feature = data['feature']
        return super().status_set_inactive(feature)


class ClassiferSimulatorMoveVehicle(ClassifierSimulatorMove):
    def __init__(self, pygame, screen):
        super().__init__(pygame, screen, road_artifact.ObjRoadArtifactMoveVehicle, 50, 'bottom')

    def get_process_data(self, status, feature):
        data = super().get_process_data(status, feature)

        car = data['status']['car']
        road = data['status']['location']['road']
        artifact = feature['artifact']

        if road.lane_cnt == 1:
            # only 1 lane - match speed of car
            data['type'] = 'single_lane'
            data['artifact_pos'] = artifact.gnav('bottom')
        else:
            # select adjoining lane
            data['type'] = 'multiple_lane'
            car.set_collision_buffer_parms('top-front')

            lane_id_current = status['location']['lane'].lane_id
            if lane_id_current - 1 >= 0:
                lane_id_new = lane_id_current - 1
            else:
                lane_id_new = lane_id_current + 1

            # create drive guide
            data['drive'] = drive_lib.DriveArcChangeLane(self.pygame, self.screen, car, road, lane_id_current, lane_id_new)
        return data

    def process_function(self, data):
        def change_lane(data):
            car = data['status']['car']
            drive = data['drive']

            target_heading = drive.get_heading(car)
            if target_heading is not None:
              return self.send_instruction(car, target_heading, car.speed, f'Changing lane to avoid slow moving vehicle', 1000)
            else:
                return self.status_set_inactive(data)

        def slow_down(data):
            car = data['status']['car']
            feature = data['feature']
            artifact = feature['artifact']
            road = data['status']['location']['road']

            pos_artifact = artifact.gnav('top') 
            pos_car = car.gnav('top')
            distance = (pos_artifact - pos_car) * road.graph_dir_length
            
            if distance > 5:
                car.slow_down()
                return self.send_instruction(car, artifact.heading, car.speed, 'Reducing speed for slow vehicle', 1000)
            elif distance < 0:
                car.acceleration = 15
                car.set_speed()
                return self.status_set_inactive(data)
            else:
                car.speed = 0
                car.acceleration = 0
                return self.send_instruction(car, None, car.speed, 'Stopped for stopped vehicle', 1000)

        ## process_function()
        if data['type'] == 'single_lane':
            return slow_down(data)
        else:
            return change_lane(data)


class ClassiferSimulatorMovePedestrian(ClassifierSimulatorMove):
    def __init__(self, pygame, screen):
        super().__init__(pygame, screen, road_artifact.ObjRoadArtifactMovePedestrian, 18, 'bottom')

    def activate(self, status, feature):
        
        if not super().activate(status, feature):
            return False
        pedestrian = feature['artifact']
        car = status['car']
        return self.in_warning_collision_buffer(car, pedestrian)

    def process_function(self, data):
        car = data['status']['car']
        pedestrian = data['feature']['artifact']
        feature = data['feature']
        artifact = feature['artifact']
        road = data['status']['location']['road']

        pos_artifact = artifact.gnav('top') 
        pos_car = car.gnav('top')
        distance = (pos_artifact - pos_car) * road.graph_dir_length
        if distance > 30:
            car.slow_down()
            return self.send_instruction(car, None, car.speed, 'Slowing down for pedestrian ahead', 1000) 
        elif distance <= 30:
            if self.in_collision_buffer(car, pedestrian):
                car.speed = 0
                car.draw_collision_buffer()
                return self.send_instruction(car, None, 0, 'Waiting for pedestrian', 1000)
            else:
                car.acceleration = 15
                car.set_speed()
                return super().status_set_inactive(data)
        
