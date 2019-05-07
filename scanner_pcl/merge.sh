#!/bin/bash

echo "Merge starting"

for (( i=0; i<=$1; ++i ))
do
    echo Processing file \#$i
    if [ $i -eq 0 ]
    then
        rosrun scanner_pcl icp data/0.pcd data/1.pcd 0.001
    else
        rosrun scanner_pcl icp data/merged$[i-1].pcd data/$i.pcd 0.001
    fi

    # Delete outliers each 5 iterations:
    if [ $[($i +1) % 5] -eq 0 ]
    then
        rosrun scanner_pcl outliers_filter data/merged.pcd 100 3.0
        echo ">>>>>>>>>>>>>>> OUTLIER REMOVING >>>>>>>>>>>>>>>>>>>>>>>>>>>"
    fi    
    mv data/merged.pcd data/merged$i.pcd
done

now=$(date +"%m_%d_%Y_%H_%M_%S")
cp data/merged$1.pcd collection/$now.pcd

echo "To look at result use:"
echo "pcl_viewer -bc 1,1,1  -multiview 1 data/merged.pcd"