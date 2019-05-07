#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>


typedef pcl::PointXYZRGBA PointT;
static double leaf_edge = 0.05;
static double MAX_ITERS = 2;

pcl::PointCloud<PointT>::Ptr loadPC(char* filename) {
	pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
	if (pcl::io::loadPCDFile<PointT> (filename, *cloud) == -1) //* load the file
	{
		PCL_ERROR ("Couldn't read file *.pcd \n");
		return NULL;
	}

	return cloud;
}


pcl::PointCloud<PointT>::Ptr downgrade(pcl::PointCloud<PointT>::Ptr cloud_ptr) {
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
    // ros::init(argc, argv, "pcl_matching");

    // cloudHandler handler;

    // ros::spin();
    std::string filename = "/home/k3dr/catkin_ws/src/scanner/scanner_pcl/data/merged.pcd";
	auto cloud2 = loadPC(argv[1]); //Target
	auto cloud1 = loadPC(argv[2]); //Source. Will be downgraded
	
	
	if(argc >= 4) {
		leaf_edge = atof(argv[3]);
	} 
	if(argc >= 5) {
		MAX_ITERS = atof(argv[4]);
	} 

	pcl::PointCloud<PointT> cloud_aligned;
	auto cloud1_downgraded = downgrade(cloud1);

	std::cout << "Loaded" << std::endl;

	// ICP
	pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(cloud1_downgraded);
    icp.setInputTarget(cloud2->makeShared());
	// More parameters here
    icp.setMaxCorrespondenceDistance(0.05); // 50mm 
    icp.setTransformationEpsilon (1e-8);
	icp.setRANSACOutlierRejectionThreshold(1.5 * leaf_edge);
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
	auto cloud_filtered = downgrade(cloud_aligned.makeShared());	
	ROS_INFO("PointCloud after filtering: %d data points (%f).",  
					cloud_filtered->width * cloud_filtered->height, pcl::getFieldsList (*cloud_filtered));
	

	
    pcl::io::savePCDFileBinary (filename, *cloud_filtered);
	
	std::cout << "Finished" << std::endl;

	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  	icp.getFitnessScore() << std::endl;
  	std::cout << icp.getFinalTransformation() << std::endl;

    return 0;
}
