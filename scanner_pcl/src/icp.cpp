#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <json.hpp>

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


pcl::PointCloud<PointT>::Ptr downgrade(pcl::PointCloud<PointT>::Ptr cloud_ptr, double leaf_edge) {
	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
	// Downsampling by VoxelGrid
	pcl::VoxelGrid<PointT> sor;
	// sor.setInputCloud (cloud_aligned);
	sor.setInputCloud (cloud_ptr);
	// TODO: make string below not hardcoded (for different sizes of objects it is matter)!
	sor.setLeafSize (leaf_edge,leaf_edge,leaf_edge); // Voxels are of size 10*10*10 mm3
	sor.filter (*cloud_filtered);

	return cloud_filtered;
}


int main(int argc, char **argv)
{
	char *conf_file = "conf/ICP.json";
	if(argc > 3){
		conf_file = argv[3];
	}

    // Read configs JSON
	std::ifstream i(conf_file);
	json conf;
	i >> conf;

    std::string filename = conf["MERGED_FILE"].get<std::string>();
	auto cloud2 = loadPC(argv[1]); //Target
	auto cloud1 = loadPC(argv[2]); //Source. Will be downgraded
	
	double leaf_edge = conf["LEAF_EDGE"].get<double>();
	double MAX_ITERS = conf["MAX_ITERS"].get<int>();
	double MAX_CORRESPONDENCE_DISTANCE = conf["MAX_CORRESPONDENCE_DISTANCE"].get<double>();
	double TRANSFORMATION_EPSILON = conf["TRANSFORMATION_EPSILON"].get<double>();
	double RANSAC_OUTLIER_REJECTION_THRESHOLD = conf["RANSAC_OUTLIER_REJECTION_THRESHOLD"].get<double>();


	pcl::PointCloud<PointT> cloud_aligned;
	auto cloud1_downgraded = downgrade(cloud1, leaf_edge);

	std::cout << "Loaded" << std::endl;

	// ICP
	pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(cloud1_downgraded);
    icp.setInputTarget(cloud2->makeShared());
	// More parameters here
    icp.setMaxCorrespondenceDistance(MAX_CORRESPONDENCE_DISTANCE); // 50mm 
    icp.setTransformationEpsilon (TRANSFORMATION_EPSILON);
	icp.setRANSACOutlierRejectionThreshold(RANSAC_OUTLIER_REJECTION_THRESHOLD);
    // icp.setEuclideanFitnessEpsilon(1e-8);
    icp.setMaximumIterations(MAX_ITERS);
	icp.align(cloud_aligned);
    std::cout << "Aligned" << std::endl;

	// Apply found transformation
	pcl::transformPointCloud (*cloud2, cloud_aligned, icp.getFinalTransformation().inverse());
	cloud_aligned += *cloud1;
	
	// Downgrade output cloud
	ROS_INFO("PointCloud before filtering: %d data points (%f).",  
					cloud_aligned.width * cloud_aligned.height, pcl::getFieldsList (cloud_aligned) );
	auto cloud_filtered = downgrade(cloud_aligned.makeShared(), leaf_edge);	
	ROS_INFO("PointCloud after filtering: %d data points (%f).",  
					cloud_filtered->width * cloud_filtered->height, pcl::getFieldsList (*cloud_filtered));
	

	
    pcl::io::savePCDFileBinary (filename, *cloud_filtered);
	
	std::cout << "Finished" << std::endl;

	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  	icp.getFitnessScore() << std::endl;
  	std::cout << icp.getFinalTransformation() << std::endl;

    return 0;
}
