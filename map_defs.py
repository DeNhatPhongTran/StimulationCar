maps = []
roads = []
artifacts = []
R = "r"
L = "l"
C = "c"

def load():
    #add_map(2, 0, load_map(map0))
    #add_map(2, 0, load_map(map1))
    #add_map(2, 0, load_map(map2))
    #add_map(2, 0, load_map(map3))

    #add_map(1, 0, load_map(map4))
    #add_map(2, 0, load_map(map5))
    #add_map(2, 0, load_map(map6))
    add_map(2, 0, load_map(map7))

    #add_map(2, 0, load_map(mapTest))

    return maps


def mapTest():
    add_road(2, .25)
    add_artifact("sign_speed_65", 2, 2, L)
    add_road(2, 1)
    add_artifact("vehicle", 2, 2, 0)
    add_artifact("destination", 0, 2, C)

def map0():
    add_road(3, .5)
    add_artifact("sign_speed_45", 2, 2, L)
    add_road(3, 1)
    add_artifact("sign_speed_65", 2, 2, L)
    add_road(3, .5)
    add_artifact("destination", 0, 2, C)

def map1():
    add_road(3, .5)
    add_artifact("sign_speed_65", 2, 2, L)
    add_artifact("sign_stop", 0, 0, R)
    add_road(3, 1)
    add_artifact("sign_stop", 1, 0, R)
    add_artifact("sign_stop", 0, 0, R)
    add_road(3, .5)
    add_artifact("destination", 0, 2, C)

def map2():
    add_road(3, .5)
    add_artifact("sign_speed_65", 2, 2, L)
    add_artifact("sign_traffic_light", 0, 2, C)
    add_road(3, 1)
    add_artifact("sign_traffic_light", 0, 2, C)
    add_road(3, .25)
    add_artifact("destination", 0, 2, C)

def map3():
    add_road(3, .75)
    add_artifact("sign_speed_65", 2, 2, L)
    add_artifact("pedestrian", 1, 0, R)
    add_artifact("pedestrian", 0, 0, L)
    add_road(3, .75)
    add_artifact("pedestrian", 1, 0, R)
    add_artifact("pedestrian", 0, 0, L)
    add_artifact("destination", 0, 2, C)

def map4():
    add_road(3, .25)
    add_artifact("sign_speed_65", 2, 2, L)
    add_road(3, .75)
    add_artifact("vehicle", 2, 2, 0)
    add_road(3, .75)
    add_artifact("vehicle", 2, 2, 0)
    add_artifact("destination", 0, 2, C)

def map5():
    add_road(3, .25)
    add_artifact("sign_speed_65", 2, 2, L)
    add_road(3, .75)
    add_artifact("vehicle", 2, 2, 0)
    add_road(3, .75)
    add_artifact("vehicle", 2, 2, 0)
    add_artifact("destination", 0, 2, C)

def map6():
    add_road(3, .25)
    add_artifact("sign_speed_65", 2, 2, L)
    add_road(3, .75)
    add_artifact("vehicle", 2, 2, 0)
    add_artifact("pedestrian", 0, 0, R)
    add_road(3, .75)
    add_artifact("vehicle", 2, 2, 1)
    add_artifact("pedestrian", 0, 0, L)
    add_artifact("destination", 0, 2, C)

def map7():
    add_road(0, 0.25)
    add_artifact("sign_speed_55", 2, 2, L)
    add_artifact("sign_traffic_light", 0, 2, C)
    add_road(2, .4)
    add_artifact("sign_stop", 0, 0, R)
    add_road(2, .4)
    add_artifact("sign_traffic_light", 0, 2, C)
    add_road(0, 1)
    add_artifact("sign_speed_65", 2, 2, R)
    add_artifact("pedestrian", 0, 1, L)
    add_road(0, 1)
    add_artifact("sign_traffic_light", 0, 2, C)
    add_artifact("pedestrian", 1, 1, L)
    add_road(3, 2.8)
    add_artifact("sign_speed_45", 2, 2, L)
    add_artifact("vehicle", 2, 1, 1)
    add_artifact("pedestrian", 0, 1, L)
    add_artifact("sign_stop", 0, 0, R)
    add_artifact("vehicle", 1, 2, 0)
    add_artifact("pedestrian", 2, 1, L)
    add_road(1, .85)
    add_artifact("sign_speed_25", 2, 2, L)
    add_artifact("pedestrian", 0, 1, R)
    add_road(2, .85)
    add_artifact("vehicle", 2, 1, 1)
    add_artifact("destination", 0, 2, C)


def map8():
    add_road(3, 2)
    add_artifact("vehicle", 1, 1, 0)
    #add_artifact("pedestrian", 1, 1, R)
    add_artifact("sign_speed_65", 2, 2, L)
    add_artifact("sign_stop", 0, 0, R)
    add_road(3, 1)
    add_artifact("destination", 0, 2, C)


#utilities
def add_map(lane_count, car_start_lane, map_parms):
    maps.append((lane_count, car_start_lane, map_parms))

def load_map(f_map):
    global roads
    roads = []
    global artifacts
    artifacts = []
    f_map()
    return roads, artifacts

def add_road(direction, length):
    global roads
    roads.append((direction, length))
    artifacts.append([])

def add_artifact(type, segment, pos_length, pos_width):
    global artifacts
    artifacts[-1].append((type, segment, pos_length, pos_width))
