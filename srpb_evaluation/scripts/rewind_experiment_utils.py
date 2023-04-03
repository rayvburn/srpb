'''
Helper functions to create a visual representing a specific experiment based on the logged data
'''

import math
import numpy as np
import csv
from typing import List
from typing import Dict
import matplotlib.cm as cm


'''
Loads and parses logged data providing handy methods to retrieve robot information
'''
class Robot:
    def __init__(self, log_data: List[str]):
        self.timestamp = float(log_data[0])
        self.x  = float(log_data[1])
        self.y  = float(log_data[2])
        self.th = float(log_data[4])

    def get_timestamp(self) -> float:
        return self.timestamp

    def get_x(self) -> float:
        return self.x

    def get_y(self) -> float:
        return self.y

    def get_orientation(self) -> float:
        return self.th


'''
Loads and parses logged data providing handy methods to retrieve person information
'''
class Person:
    def __init__(self, log_data: List[str]):
        # check if only timestamp is available
        if len(log_data) == 1:
            self.dummy = True
            return
        self.dummy = False

        # parse
        self.timestamp = float(log_data[0])
        self.name = str(log_data[1])
        self.x = float(log_data[3])
        self.y = float(log_data[4])
        self.th = float(log_data[6])

    def is_dummy(self) -> bool:
        return self.dummy

    def get_timestamp(self) -> float:
        return self.timestamp

    def get_id(self) -> str:
        return self.name

    def get_x(self) -> float:
        return self.x

    def get_y(self) -> float:
        return self.y

    def get_orientation(self) -> float:
        return self.th


'''
Loads and parses logged data providing handy methods to retrieve group information
'''
class Group:
    def __init__(self, log_data: List[str]):
        # check if only timestamp is available
        if len(log_data) == 1:
            self.dummy = True
            return
        self.dummy = False

        # NOTE: group log contains 2 parts divided by '/'
        group_data = []
        relations_data = []
        relations_data_proc = False
        for entry in log_data:
            if entry == '/':
                relations_data_proc = True
                continue
            if not relations_data_proc:
                group_data.append(entry)
            else:
                relations_data.append(entry)

        # parse
        self.timestamp = float(group_data[0])
        self.name = group_data[1]
        self.cog_x = float(group_data[2])
        self.cog_y = float(group_data[3])
        self.cog_z = float(group_data[4])
        self.age = int(group_data[5])
        # check if any members available
        self.member_ids = []
        self.relations = []
        if len(group_data) == 6:
            return
        # collect member ids
        for member_id in group_data[6:]:
            self.member_ids.append(member_id)
        # collect relations
        # ref: https://renanmf.com/python-for-loop-increment-by-2/
        relation_indexes = range(0, len(relations_data))
        for i in relation_indexes[0::3]:
            relation = (str(relations_data[i]), str(relations_data[i+1]), float(relations_data[i+2]))
            self.relations.append(relation)

    def is_dummy(self) -> bool:
        return self.dummy

    def get_timestamp(self) -> float:
        return self.timestamp

    def get_id(self) -> str:
        return self.name

    def get_x(self) -> float:
        return self.cog_x

    def get_y(self) -> float:
        return self.cog_y

    def get_member_ids(self) -> List[str]:
        return self.member_ids

    def get_relations(self) -> List[tuple]:
        return self.relations

    def get_age(self) -> int:
        return self.age


def read_csv_with_multiple_delimiters_in_line(log_file: str) -> List[str]:
    file = open(log_file, 'r')
    log_raw = list(csv.reader(file, delimiter=' '))

    log_clean = []
    for i, line in enumerate(log_raw):
        line_clean = []
        for j, entry in enumerate(line):
            if not len(entry):
                continue
            line_clean.append(entry)
        log_clean.append(line_clean)
    return log_clean


def compute_map_origin(mapfull, crop_area):
    # NOTE: 'axes' may be inverted
    origin_imgfull  = [len(mapfull) / 2.0, len(mapfull[0]) / 2.0]
    origin_imgcrop = [origin_imgfull[0] - crop_area[0], origin_imgfull[1] - crop_area[1]]
    return origin_imgcrop


'''
Recomputes center of gravity (COG) based on valid members of the group
'''
def recompute_group_cog(group: Group, people_set: List[Person], people_hide: List[str]) -> Group:
    # extract people for the timestamp of the group
    valid_people_this_group = []
    for person in people_set:
        if person.get_timestamp() != group.get_timestamp():
            continue
        if person.get_id() in people_hide:
            continue
        if not person.get_id() in group.get_member_ids():
            continue
        valid_people_this_group.append(person)

    # compute new COG
    new_cog_x = 0.0
    new_cog_y = 0.0
    for person in valid_people_this_group:
        new_cog_x += person.get_x()
        new_cog_y += person.get_y()
    new_cog_x /= len(valid_people_this_group)
    new_cog_y /= len(valid_people_this_group)
    new_cog_z = 0.0

    # collect valid member IDs
    new_member_ids = [] # list of strings
    for member_id in group.get_member_ids():
        if member_id in people_hide:
            continue
        new_member_ids.append(member_id)

    # collect valid relations
    new_relations = [] # list of tuples
    for relation in group.get_relations():
        if relation[0] in people_hide or relation[0] in people_hide:
            continue
        new_relations.append(relation)

    # create a list to construct a new group from - simulates converting group to string
    new_group_args = [] # list of strings
    new_group_args.append(str(group.get_timestamp()))
    new_group_args.append(str(group.get_id()))
    new_group_args.append(str(new_cog_x))
    new_group_args.append(str(new_cog_y))
    new_group_args.append(str(new_cog_z))
    new_group_args.append(str(group.get_age()))
    for member_id in new_member_ids:
        new_group_args.append(str(member_id))
    new_group_args.append('/')
    for relation in new_relations:
        new_group_args.append(str(relation[0]))
        new_group_args.append(str(relation[1]))
        new_group_args.append(str(relation[2]))
    return Group(new_group_args)


def extract_timestamps(robot_data: List[Robot], timestamp_max: float) -> np.array:
    ts_first = robot_data[0].get_timestamp()
    # choose last timestamp
    ts_last  = robot_data[-1].get_timestamp()
    if not timestamp_max == None and timestamp_max > ts_last:
        ts_last = timestamp_max
    # determined by the accuracy of time stamps in logs
    step = 1e-04
    timestamps = np.arange(ts_first, ts_last + step, step)
    # trim to 4 digits - see `step` note
    for i, ts in enumerate(timestamps):
        timestamps[i] = round(ts, ndigits=4)
    return timestamps


def get_colormap():
    return cm.rainbow


'''
Same colormap must be used as returned from @ref get_colormap
'''
def prepare_colormap_for_timestamps(timestamps: List, alpha: float = 0.5) -> Dict[float, List[float]]:
    # creates color for each timestamp in the list
    colors = cm.rainbow(np.linspace(0, 1, len(timestamps)))
    # creates colormap for each timestamp
    colormap = {float: List[float]}
    for i, ts in enumerate(timestamps):
        color = colors[i]
        color[3] = alpha
        colormap[float(ts)] = color
    return colormap


def rotz(theta):
    return np.matrix([[math.cos(theta), -math.sin(theta), 0 ],
                      [math.sin(theta),  math.cos(theta), 0 ],
                      [0           , 0            , 1 ]])


def transform_logged_pos_to_map(p_in: np.array, R: np.matrix, t: np.array) -> List[float]:
    T = [
        [R[0,0], R[0,1], R[0,2], t[0]],
        [R[1,0], R[1,1], R[1,2], t[1]],
        [R[2,0], R[2,1], R[2,2], t[2]],
        [0,      0,      0,      1]
    ]
    p_inT = np.array([[p_in[0]], [p_in[1]], [p_in[2]], [1.0]])
    p_out = np.matmul(T, p_inT)
    return p_out[0:2]


'''
wx, wy: world coordinates
ox, oy: map origin's image coordinates
res: map resolution
'''
def world_to_map_no_bounds(wx, wy, ox, oy, res) -> List[float]:
    mx = ox + wx / res
    my = oy + wy / res
    return [mx, my]


'''
Similar to world_to_map_no_bounds but for image coords
'''
def world_to_map_no_bounds_image(wx, wy, ox, oy, res) -> List[float]:
    mx = ox + wx / res
    my = oy - wy / res
    return [mx, my]

