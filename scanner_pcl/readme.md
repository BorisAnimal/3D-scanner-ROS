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