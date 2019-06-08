import json
import os
import argparse

### List of tuples. List - for each algorithm (hence for each .json file), 
### tuple for concrete parameter for this algorithm.
ICP_PARAMS = [
    # (flag_name, default_val, help_str)

    ## Flag --ICP_FILENAME not used yet, because for each algo it will be universal
    ## ('--ICP_FILENAME', 'icp.json', 'file where configs for icp algorithm will be stored'),
    ('--LEAF_EDGE', 0.05, 
                    "Downsampling. Voxel's edge size"),
    ('--MAX_ITERS', 10, 
                    "ICP. Maximum iterations convergence criterium"),
    ('--MAX_CORRESPONDENCE_DISTANCE', 0.05,
                    "ICP. Max dist between members to take them into account than computing error"),
    ('--TRANSFORMATION_EPSILON', 1e-8,
                    "ICP. Convergence criterium"),
    ('--RANSAC_OUTLIER_REJECTION_THRESHOLD', 0.75,
                    "ICP. RANSAC Outlier Rejection Threshold")
]

### Dict of all parameter-lists. Like registration
ALL_PARAMS = {
    "ICP": ICP_PARAMS
}


### Init parser. As parameter for this algorithm there are --conf_dir flag.
parser = argparse.ArgumentParser(description='Configs generator')
parser.add_argument('--conf_dir', default="../conf/", help="Directory where all .json param files are stored.")

for algorithm, params in ALL_PARAMS.items():
    for param in params:
        parser.add_argument(param[0], default=param[1], type=type(param[1]), help=param[2])
        



def main(args=parser.parse_args()):
    for algorithm, params in ALL_PARAMS.items():
        data = {}
        for param in params:
            name = param[0].replace("--","")
            data[name] = getattr(args, name)
    
        filename = os.path.join(args.conf_dir, algorithm+".json")

        with open(filename, "w") as write_file:
            json.dump(data, write_file)
    pass

if __name__ == "__main__":
    main()