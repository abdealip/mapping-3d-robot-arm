#!/usr/bin/python3

import os
import json
import numpy as np
from argparse import ArgumentParser

from stl_util import STL

if __name__ == "__main__":
    parser = ArgumentParser()

    repo_dir = os.path.dirname(os.path.dirname(__file__))
    env_json_file = os.path.join(repo_dir, "config", "environment_corrected.json")
    parser.add_argument("-e", "--env", default=env_json_file, help="environment json file")
    parser.add_argument("-o", "--output", required=True, help="output STL file")
    options = parser.parse_args()
    with open(options.env) as f:
        env_json = json.load(f)

    combined_model = None

    for mesh in env_json["meshes"]:
        model = STL(os.path.join(repo_dir, "models", mesh["model"]))
        model.transform(np.array(mesh["transform"]))
        if combined_model == None:
            combined_model = model
        else:
            combined_model.append_other_stl_data(model)

    # apply transform from mapping frame to rviz frame
    combined_model.transform(np.array(env_json["global_transform"]))

    combined_model.write(options.output)
