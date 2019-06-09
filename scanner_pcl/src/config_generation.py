import json
import os
from os.path import dirname, join
import sys
import argparse


def get_parent_path():
    return dirname(dirname(join(os.getcwd(), sys.argv[0])))

### List of tuples. List - for each algorithm (hence for each .json file), 
### tuple for concrete parameter for this algorithm.
ICP_PARAMS = [
    # (flag_name, default_val, help_str)

    ## Flag --ICP_FILENAME not used yet, because for each algo it will be universal
    ## ('--ICP_FILENAME', 'icp.json', 'file where configs for icp algorithm will be stored'),
    ('--LEAF_EDGE', 0.05, 
                    """Downsampling. Voxel's edge size. Affects on density of points. 
                    Smaller object - higher density. Bigger object - lower."""),
    ('--MAX_ITERS', 10, 
                    "ICP. Maximum iterations convergence criterium"),
    ('--MAX_CORRESPONDENCE_DISTANCE', 0.05,
                    "ICP. Max dist between members to take them into account than computing error"),
    ('--TRANSFORMATION_EPSILON', 1e-8,
                    "ICP. Convergence criterium"),
    ('--RANSAC_OUTLIER_REJECTION_THRESHOLD', 0.75,
                    "ICP. RANSAC Outlier Rejection Threshold"),
                    ## Below is dirty way to get path to current file and to append relative path
    ('--MERGED_FILE', join(get_parent_path(), "data/merged.pcd"),
                    "Path to file, where result of ICP will be saved")
]

OUTLIERS_FILTER = [
    ("--MEAN_K", 50,
                    "Main filtering parameter"),
    ("--STDDEV_MUL_THRESH", 1.0, 
                    "Starndart deviation multiptier (for threshold)")
]

ROTATE_AND_SCAN = [
    ("--ROTATION_EPSILON", 0.05, ""),
    ("--FILTER_X_MIN", -10.0, ""),
    ("--FILTER_X_MAX", 10.0, ""),
    ("--FILTER_Y_MIN", -10.0, "due to our scanner relatively reversed"),
    ("--FILTER_Y_MAX", 0.349, "due to our scanner relatively reversed"),
    ("--FILTER_Z_MIN", 0.5, ""),
    ("--FILTER_Z_MAX", 2.0, ""),
    ("--TRANSFORM_WAIT_DURATION", 1.5, ""),
    ("--DATA_FOLDER", join(get_parent_path(), "data/"), ""),
    ("--SCANNER_LINK", "cameraLeft_depth_link", ""),
    ("--BASE_LINK", "world", ""),
    ("--CONTROLLER_TOPIC", "/scanner/joint0_position_controller/command", ""),
    ("--PC_TOPIC", "/camera/depth/points", ""),
    ("--JOINT_STATES_TOPIC", "/scanner/joint_states", ""),
    ("--SCANNING_MEAN_K", 50, ""),
    ("--LOOP_RATE", 25, ""),
    ("--SCANNING_STDDEV_MUL_THRESH", 1.0, "")
]

### Dict of all parameter-lists. Like registration
ALL_PARAMS = {
    "ICP": ICP_PARAMS,
    "OUTLIERS_FILTER": OUTLIERS_FILTER,
    "ROTATE_AND_SCAN": ROTATE_AND_SCAN
}


### Init parser. As parameter for this algorithm there are --conf_dir flag.
parser = argparse.ArgumentParser(description='Configs generator')
parser.add_argument('--conf_dir', default=join(get_parent_path(), "conf/"), help="Directory where all .json param files are stored.")
parser.add_argument('--show_usage', default=0, type=int, help="Prints, what to write in C++ for all arguments")

for algorithm, params in ALL_PARAMS.items():
    for param in params:
        parser.add_argument(param[0], default=param[1], type=type(param[1]), help=param[2])
        

mapping = {
    str : "std::string",
    float : "double",
    int: "int",
    bool: "bool"
    # idk what for list and tuple. See NLohmann documentation.
}


def main(args=parser.parse_args()):
    for algorithm, params in ALL_PARAMS.items():
        if args.show_usage:
            print("\nFor algorithm ({}) use: \n".format(algorithm))
            print('#include "json.hpp"')
            print("using json = nlohmann::json;\n")
        static = []
        definition = []
        data = {}
        for param in params:
            name = param[0].replace("--","")
            data[name] = getattr(args, name)
            if args.show_usage:
                static.append('static {} {};'.format(mapping[type(data[name])], name))
                definition.append('{} = conf["{}"].get<{}>();'.format(name,name, mapping[type(data[name])]))
    
        if args.show_usage:
            print("\n".join(static))
            print("")
            print("// Read configs JSON")
            print("std::ifstream i(conf_file);")
            print("json conf;")
            print("i >> conf;")
            print("")
            print("\n".join(definition))
            print("=============================")

        filename = os.path.join(args.conf_dir, algorithm+".json")
        if not os.path.exists(args.conf_dir):
            os.makedirs(args.conf_dir)

        with open(filename, "w") as write_file:
            json.dump(data, write_file, indent=2)
    pass

if __name__ == "__main__":
    main()