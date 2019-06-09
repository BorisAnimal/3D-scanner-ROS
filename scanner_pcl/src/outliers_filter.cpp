#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>

#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <Eigen/Eigen>

#include "json.hpp"

using json = nlohmann::json;

typedef pcl::PointXYZRGBA PointT;

pcl::PointCloud<PointT>::Ptr loadPC(char* filename) {
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
	if (pcl::io::loadPCDFile<PointT> (filename, *cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file *.pcd \n");
		return NULL;
	}

	return cloud;
}

// Warning! This function overwrites file!
int main (int argc, char **argv) {
    char *conf_file = "conf/OUTLIERS_FILTER.json";
	if(argc > 2){
		conf_file = argv[2];
	}

    // Read configs JSON
	std::ifstream i(conf_file);
	json conf;
	i >> conf;

    int meanK = conf["MEAN_K"].get<int>();
    double mult = conf["STDDEV_MUL_THRESH"].get<double>();

    auto cloud = loadPC(argv[1]); // Ptr type    

    PCL_INFO("Size before filtering: %d\n", cloud->size());

    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (meanK);
    sor.setStddevMulThresh (mult);
    sor.filter (*cloud);

    PCL_INFO("Outliers filtered. %d\n", cloud->size());

    pcl::io::savePCDFileBinary (argv[1], *cloud);
}