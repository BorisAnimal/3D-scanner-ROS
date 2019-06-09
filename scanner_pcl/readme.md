#  To visualize .pcd file (need not-default package to be installed)
pcl_viewer -multiview 1 ~/catkin_ws/src/scanner_pcl/data/merged.pcd 

To parse .pcd to .ply
pcl_pcd2ply kek.pcd kek.ply


Now we have cool config generator (src/config_generation.py). Here are general configs
for each of algorithms. Here you can add your or just modify defaults during python
call:
```bash
# here --MAX_ITERS 5 modifies default value (10) of parameter MAX_ITERS of ICP to 5.
python src/config_generation.py --MAX_ITERS 5 
```
Then you can easily modify gerenated file manually in conf/ directory



How to contribute:
1) If you need to add .cpp in compilation pipeline - append it in CMakeLists.txt
with respect to examples already here. Pay attention: some C++ specific 
dependencies (such as pcl_conversions I met) must be provided in this file as 
well. You will meet this with specific errors during compilation.

2) All parameters and configurations try to provide in code via config_generation.py 
and .json - config files. Here you can append derive parameters logic (for example
try to guess values after bounding box scanning rotation).

3) Scanning realized in 
```
rosrun scanner_pcl rotate_and_scan [Num of pos]
```

Merging is in
```
. merge.sh [num of files starting from 0] 
```
This is main file of pipeline logic.

# TODO: put pictures from presentation.
