'''
Creates a visual representing a specific experiment based on the logged data

Dependencies:
sudo apt install python3-matplotlib
sudo apt install python3-scipy
sudo apt install python3-numpy

'''
import sys
import yaml

import math
import matplotlib as mpl
import matplotlib.pyplot as plt
import matplotlib.patches as ptch
import numpy as np

from scipy import ndimage
from typing import List
from typing import Dict

# import from the local file
# classes
from rewind_experiment_utils import Robot
from rewind_experiment_utils import Person
from rewind_experiment_utils import Group
# functions
from rewind_experiment_utils import read_csv_with_multiple_delimiters_in_line
from rewind_experiment_utils import recompute_group_cog
from rewind_experiment_utils import extract_timestamps
from rewind_experiment_utils import rot_mat_to_euler_angles
from rewind_experiment_utils import transform_logged_pos_to_map
from rewind_experiment_utils import compute_transform
from rewind_experiment_utils import rotz
from rewind_experiment_utils import compute_map_origin
from rewind_experiment_utils import world_to_map_no_bounds_image
from rewind_experiment_utils import prepare_colormap_for_timestamps
from rewind_experiment_utils import get_colormap


def main(
    timestamps: np.array,
    log_data: Dict[str, List],
    extra_pts: List[List[float]],
    tf_map_logged: List[float],
    map,
    map_origin: List[float],
    map_resolution: float,
    visuals: Dict,
    vis_name: str
):
    # rotation matrix for log-frame -> map transform composition
    R = rotz(tf_map_logged[-1])
    # translation vector for log-frame -> map transform composition
    t = np.array([tf_map_logged[0], tf_map_logged[1], tf_map_logged[2]])

    # containers to store map coords
    robot_pos_img =  {'ts': [], 'x': [], 'y': [], 'color': []}
    people_pos_img = {'ts': [], 'id': [], 'x': [], 'y': [], 'color': []}
    groups_pos_img = {'ts': [], 'id': [], 'x': [], 'y': [], 'color': []}

    # prepare_colormap_for_timestamps
    colormap = prepare_colormap_for_timestamps(timestamps)

    # transform poses to map and image coordinates
    for robot in log_data["robot"]:
        p_in = np.array([robot.get_x(), robot.get_y(), robot.get_orientation()])
        map_pos = transform_logged_pos_to_map(p_in, R, t)
        img_pos = world_to_map_no_bounds_image(map_pos[0], map_pos[1], map_origin[0], map_origin[1], map_resolution)
        robot_pos_img['ts'].append(robot.get_timestamp())
        robot_pos_img['x'].append(float(img_pos[0]))
        robot_pos_img['y'].append(float(img_pos[1]))
        robot_pos_img['color'].append(colormap[robot.get_timestamp()])

    for person in log_data["people"]:
        p_in = np.array([person.get_x(), person.get_y(), person.get_orientation()])
        map_pos = transform_logged_pos_to_map(p_in, R, t)
        img_pos = world_to_map_no_bounds_image(map_pos[0], map_pos[1], map_origin[0], map_origin[1], map_resolution)
        people_pos_img['ts'].append(person.get_timestamp())
        people_pos_img['id'].append(person.get_id())
        people_pos_img['x'].append(float(img_pos[0]))
        people_pos_img['y'].append(float(img_pos[1]))
        people_pos_img['color'].append(colormap[person.get_timestamp()])

    for group in log_data["groups"]:
        p_in = np.array([group.get_x(), group.get_y(), 0.0])
        map_pos = transform_logged_pos_to_map(p_in, R, t)
        img_pos = world_to_map_no_bounds_image(map_pos[0], map_pos[1], map_origin[0], map_origin[1], map_resolution)
        groups_pos_img['ts'].append(group.get_timestamp())
        groups_pos_img['id'].append(group.get_id())
        groups_pos_img['x'].append(float(img_pos[0]))
        groups_pos_img['y'].append(float(img_pos[1]))
        groups_pos_img['color'].append(colormap[group.get_timestamp()])

    # visualisation
    fig, ax = plt.subplots()
    # that will make the UI run in a separate thread and the call to show will return immediately
    # plt.ion()

    # show map in greyscale
    plt.imshow(map, cmap=plt.get_cmap('gray'))

    # plot coords of robot, people and groups (1 by 1 to be able to select color per each point)
    for i, _ in enumerate(robot_pos_img['x']):
        plt.plot(robot_pos_img['x'][i], robot_pos_img['y'][i], 'o', color=robot_pos_img['color'][i])

    for i, _ in enumerate(people_pos_img['x']):
        center = people_pos_img['x'][i], people_pos_img['y'][i]
        ec = visuals['people']['ec']
        radius = visuals['people']['radius']
        c = ptch.Circle(center, radius=radius, edgecolor=ec, facecolor=people_pos_img['color'][i])
        plt.gca().add_patch(c)

    for i, _ in enumerate(groups_pos_img['x']):
        center = groups_pos_img['x'][i], groups_pos_img['y'][i]
        # use the color according to timestamp, change alpha
        ec = groups_pos_img['color'][i].copy()
        ec[3] = visuals['groups']['edge_alpha']
        fc = groups_pos_img['color'][i].copy()
        fc[3] = visuals['groups']['face_alpha']
        radius = visuals['groups']['radius']
        c = ptch.Circle(center, radius=radius, linestyle='-', fill=True, edgecolor=ec, facecolor=fc)
        plt.gca().add_patch(c)

    # debugging or finding map reference points
    for pt in extra_pts:
        pt_map = transform_logged_pos_to_map(np.array([pt[0], pt[1], 0.0]), R, t)
        img_pt_map = world_to_map_no_bounds_image(pt_map[0], pt_map[1], map_origin[0], map_origin[1], map_resolution)
        # small circles but drawn on top of other entities
        plt.plot(img_pt_map[0], img_pt_map[1], 'o', ms=3)

    # disable axes scales showing image coordinates
    ax.get_xaxis().set_visible(False)
    ax.get_yaxis().set_visible(False)

    # save figure with the experiment data
    output_filename = vis_name + '.pdf'
    fig.savefig(output_filename, bbox_inches='tight', pad_inches=0)
    print(f"Logged experiment data saved to `{output_filename}` file")

    # blocking call until window with the plot is closed
    plt.show()

    # create extra figure showing passage of time in terms of colors
    # NOTE: passing figsize=(1, 9) to subplots works but only within some bounds of height and width
    fig_tim_scale, ax_tim_scale = plt.subplots()
    # adjust size
    plt.gcf().set_size_inches(visuals['legend']['width'], visuals['legend']['height'])
    colorbar = mpl.colorbar.ColorbarBase(
        ax_tim_scale,
        cmap=get_colormap(),
        norm=mpl.colors.Normalize(vmin=timestamps[0], vmax=timestamps[-1]),
        orientation='vertical'
    )
    # https://stackoverflow.com/a/67438742
    colorbar.ax.tick_params(labelsize=visuals['legend']['fontsize'])
    colorbar.set_label('Seconds', fontsize=visuals['legend']['fontsize'])

    # Save the auxiliary figure
    tim_scale_filename = vis_name + '_timing' + '.pdf'
    fig_tim_scale.savefig(tim_scale_filename, bbox_inches='tight', pad_inches=0)
    print(f"Figure with colorized passage of time saved to `{tim_scale_filename}` file")

    # show the timing legend figure
    # plt.show()


if __name__ == '__main__':
    if len(sys.argv) == 1 or len(sys.argv) > 2:
        print(f"Usage: ")
        print("")
        print(f"  {sys.argv[0]}  <full path to the config file>")
        print("")
        print(f"See config template at `rewind_experiment.yaml`")
        exit()

    # load script parameters
    config_file_path = sys.argv[1]
    with open(config_file_path) as f:
        config = yaml.safe_load(f)

    map_file = config['map']['file']
    # upper left bottom right - use when resultant map should be smaller
    crop_area = config['map']['crop_area']
    map_resolution = config['map']['resolution']
    # coordinate systems transformation to align map
    tf_map_rot = config['map']['tf_rot']
    # additional points to be drawn
    extra_pts = config['map']['additional_pts']

    log_file_basename = config['log']['file_basename']
    # coordinate systems transform from global to logged (odom)
    tf_map_logged = config['log']['tf_to_map']
    # hide perception's false-positives
    id_hide = {}
    id_hide['people'] = config['log']['hide_entity_id']['people']
    id_hide['groups'] = config['log']['hide_entity_id']['groups']
    # useful when color normalization is required between multiple trials
    timestamp_max = config['log']['max_timestamp']

    # load visual configuration
    visuals = {'people': {}, 'groups': {}, 'legend': {}}
    visuals['people']['ec'] = config['vis']['people']['edge_color']
    visuals['people']['radius'] = config['vis']['people']['radius']
    visuals['groups']['edge_alpha'] = config['vis']['groups']['edge_alpha']
    visuals['groups']['face_alpha'] = config['vis']['groups']['face_alpha']
    visuals['groups']['radius'] = config['vis']['groups']['radius']
    visuals['legend']['width'] = config['vis']['legend']['width']
    visuals['legend']['height'] = config['vis']['legend']['height']
    visuals['legend']['fontsize'] = config['vis']['legend']['fontsize']

    log_file_robot = log_file_basename + '_robot.txt'
    log_file_people = log_file_basename + '_people.txt'
    log_file_groups = log_file_basename + '_groups.txt'
    # NOTE: .split('/')[-1] to save in the script's folder
    vis_name = log_file_basename + '_rewind'

    # load map file
    if map_file:
        with open(map_file, 'rb') as pgmf:
            # https://stackoverflow.com/a/70051618
            map_raw = plt.imread(pgmf)
    else:
        map_raw = np.zeros([1, 1, 3], dtype=np.uint8)
        map_raw.fill(255)

    # rotate map image
    map_rot = ndimage.rotate(map_raw, math.degrees(tf_map_rot[-1]), reshape=True)
    # make crop_area usable once empty
    if not len(crop_area):
        crop_area.append(0)
        crop_area.append(0)
        crop_area.append(len(map_rot))
        crop_area.append(len(map_rot[0]))
    # crop map image
    map = map_rot[
        crop_area[1]:(crop_area[1]+crop_area[3]),
        crop_area[0]:(crop_area[0]+crop_area[2]),
    ]

    print(f"Shape of the rotated map: ({len(map_rot)}, {len(map_rot[0])}) px")
    print(f"Shape of the cropped map: ({len(map)}, {len(map[0])}) px")
    map_origin = compute_map_origin(map_rot, crop_area)

    # knowing rotation of the map image, adjust the logged->map image frame TF
    T = compute_transform(tf_map_rot[0:3], tf_map_rot[3:6], tf_map_logged[0:3], tf_map_logged[3:6])
    rpy = rot_mat_to_euler_angles(T[0:3,0:3])
    if abs(rpy[0]) >= 1e-05 or abs(rpy[1]) >= 1e-05:
        print("Resultant roll and pitch angles are far from 0.0, drawings will not be accurate!")
    # save TF from logged frame to the map frame (regarding rotation of the map image too)
    tf_map_img_logged = [T[0,3], T[1,3], T[2,3], rpy[0], rpy[1], rpy[2]]
    print(f"Resultant transform of logged points onto map image is: {tf_map_img_logged} (X-Y-Z-R-P-Y)")

    # load logged data
    log_data = {}
    log_data['robot']  = read_csv_with_multiple_delimiters_in_line(log_file_robot)
    log_data['people'] = read_csv_with_multiple_delimiters_in_line(log_file_people)
    log_data['groups'] = read_csv_with_multiple_delimiters_in_line(log_file_groups)
    print(f"Got {len(log_data['robot'])} samples of robot data")
    print(f"Got {len(log_data['people'])} samples of people data")
    print(f"Got {len(log_data['groups'])} samples of groups data")

    # convert logged data to recognizable classes
    log_entities = {}

    # robot
    log_entities['robot'] = []
    for entry in log_data['robot']:
        log_entities['robot'].append(Robot(entry))

    # people
    log_entities['people'] = []
    for entry in log_data['people']:
        person = Person(entry)
        if person.is_dummy():
            continue
        # add only those that are should be visible
        if person.get_id() in id_hide['people']:
            continue
        # able to add to container
        log_entities['people'].append(person)

    # groups
    log_entities['groups'] = []
    for entry in log_data['groups']:
        group = Group(entry)
        if group.is_dummy():
            continue
        # add only those that are should be visible
        if group.get_id() in id_hide['groups']:
            continue
        # re-compute groups with people that should not be visible (false-positives)
        recompute_group = False
        for person_to_hide in id_hide['people']:
            if person_to_hide in group.get_member_ids():
                recompute_group = True
                break
        if recompute_group:
            group = recompute_group_cog(group, log_entities['people'], id_hide['people'])
        # able to add to container
        log_entities['groups'].append(group)

    # create timestamps
    timestamps = extract_timestamps(log_entities['robot'], timestamp_max)
    print(f"Prepared timestamps from {timestamps[0]} to {timestamps[-1]} with {len(timestamps)} entries")

    # visualise entities on the map
    main(timestamps, log_entities, extra_pts, tf_map_img_logged, map, map_origin, map_resolution, visuals, vis_name)
