#!/usr/bin/python3

import os
from mapper import Mapper

base_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)))
config_file = os.path.join(base_dir, "config", "map_config.json")

maps = [
    "2cm_100ms_0deg.map",
    "2cm_100ms_1deg.map",
    "2cm_100ms_2deg.map",
    "2cm_100ms_3deg.map",
    "2cm_100ms_5deg.map"
]

maps1 = [
    "2cm_100ms_0deg.map",
    "2cm_250ms_0deg.map",
    "2cm_500ms_0deg.map",
    "2cm_750ms_0deg.map",
    "2cm_1000ms_0deg.map"
]

for mapfile in maps:
    map = Mapper(config_file)
    map.read_from_file(os.path.join(base_dir, "data", mapfile))
    print("file:", mapfile)
    print("number of poses:", map.get_num_poses_consumed())
    print("number of free cells:", map.get_num_free_cells())
    print()
