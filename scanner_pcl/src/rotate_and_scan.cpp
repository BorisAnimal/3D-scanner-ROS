#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <Eigen/Eigen>
#include <math.h>

#include <json.hpp>


using json = nlohmann::json;


// auto abc = json::parse();

// Constants
# define PI M_PI// 3.14159265358979323846 /* pi */

// Predefined types
typedef pcl::PointXYZRGBA PointT;
pcl::PointCloud<PointT>::Ptr
cloud_filter(pcl::PointCloud<PointT>::Ptr &cloud);

// Objects
static ros::Publisher joint_msg_pub;

// Params
static double expected_pos = 0.0;
static double increment;
static int N;
static double epsilon = 1e-3;
static int pcd_index = -1;
// Filter params
static double x_min = -10.0;
static double x_max = 10.0;
static double y_min = -10.0;
static double y_max = 0.349;
static double z_min = 0.5;
static double z_max = 2.0;

// Flags
static int position_num = 0;



void pubNextPos(double expected_pos) {
	// sensor_msgs::JointState joint_msg;
	// joint_msg.name.push_back("joint0");
	std_msgs::Float64 f;
	f.data = expected_pos;

	ROS_INFO("PUBLISHER (next_pos) :: Move to next position: %f", expected_pos);

	// joint_msg.position.push_back(expected_pos);
	joint_msg_pub.publish(f);
}


void pubNextPos() {
	expected_pos += increment;
	pubNextPos(expected_pos);
}


void waiter_callback(const sensor_msgs::JointState::ConstPtr &joint)
{
    ROS_INFO("CALLBACK (waiter) :: Joint: %f, epxected_pos: %f", joint->position[0], expected_pos);

    if(joint->position[0] - expected_pos <= epsilon) {
    	// Flag up
    	position_num = round(joint->position[0] / increment);
      // Need to make state 0.0 initial point
      if(pcd_index == -1) {
        pcd_index = 0;
      }
    } else {
      pubNextPos(expected_pos);
    }
    
}


void filter_callback(sensor_msgs::PointCloud2 cloud_raw)
{
  ROS_INFO("CALLBACK (filter) :: position_num: %d pcd_index: %d", position_num, pcd_index);
	if(position_num >= pcd_index){
	    // cloud_raw is PC data from Kinect V2;
	    pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT>);
	    std::string filename = "/home/k3dr/catkin_ws/src/scanner_pcl/data/" + std::to_string(pcd_index) + ".pcd";

	    ROS_INFO("Processing #%i PointCloud...", pcd_index);

	    // change PC format from PointCloud2 to pcl::PointCloud<PointT>
	    pcl::fromROSMsg(cloud_raw, *cloud_ptr);

	    // crop, segment, filter
	    cloud_ptr = cloud_filter(cloud_ptr);

	    // save PCD file to local folder // TODO am I really need to store it on drive?	    
	    pcl::io::savePCDFileBinary (filename, *cloud_ptr);	

	    // // Publish processed PC code
	    // sensor_msgs::PointCloud2 output;
	    // pcl::toROSMsg(*cloud_ptr, output);
	    // ROS_INFO("Processed size: %d", cloud_ptr->size());
	    // pcl_pub.publish(output);

	    // gotDataFlag = 1;
	    ++pcd_index;
	    pubNextPos();
	}
}


int main (int argc, char **argv)
{
	// INIT
	ros::init (argc, argv, "rotate_and_scan");
	ros::NodeHandle nh;
	joint_msg_pub = nh.advertise<std_msgs::Float64>("/scanner/joint0_position_controller/command", 5);
	N = atoi(argv[1]);
	increment = 2 * PI / N;

	// start working
	// 0) Go to zero pos
  ros::Rate loop_rate(10); // Hz
  expected_pos = 0.0;
	pubNextPos(expected_pos);
  loop_rate.sleep();

	// 1) init rotations
	ros::Subscriber filter_sub = nh.subscribe("/camera/depth/points", 1 , filter_callback);
	ros::Subscriber sub = nh.subscribe("/scanner/joint_states", 1 , waiter_callback);

	
	while (ros::ok())
    {
    	/* Everything must be OKay. By documentation: 
    	 * 1) By default there are only one queue for callbacks
    	 * 2) In queue for callback there are the freshest messages
    	 * 3) spinOnce force every subscriber to process its queue
    	 * So, seems like there must not be race condition and IPC troubles (I hope.)
    	 */
        ros::spinOnce();
        loop_rate.sleep();
        if(N - pcd_index == 0) {
        	ROS_INFO("Scanning finished!");
        	break;
        }
    }

	return 0;
}




///////////////////////////////////////////////////////////////
//////////////////// SUPPORT FUNCTIONS ////////////////////////
///////////////////////////////////////////////////////////////

pcl::PointCloud<PointT>::Ptr cloud_filter(pcl::PointCloud<PointT>::Ptr &cloud)
{
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

    
  //****************************************************//
    // Create the filtering object - passthrough
    pcl::PassThrough<PointT> passz;
    passz.setInputCloud (cloud);
    passz.setFilterFieldName ("z");
    passz.setFilterLimits (z_min, z_max);
    // passz.setFilterLimits (0.5, 1.5);

    // passz.setFilterLimits (-2.0, 4.0);
    //pass.setFilterLimitsNegative (true);
    passz.filter (*cloud_filtered);
    ROS_INFO("Z filtered. %d", cloud_filtered->size());

    pcl::PassThrough<PointT> passy;
    passy.setInputCloud (cloud_filtered);
    passy.setFilterFieldName ("y");
    passy.setFilterLimits (y_min, y_max);
    // passy.setFilterLimits (-0.5, 0.5);

    // passy.setFilterLimits (-2.0, 2.0);
    //pass.setFilterLimitsNegative (true);
    passy.filter (*cloud_filtered);
    ROS_INFO("Y filtered. %d", cloud_filtered->size());

    pcl::PassThrough<PointT> passx;
    passx.setInputCloud (cloud_filtered);
    passx.setFilterFieldName ("x");
    passx.setFilterLimits (x_min, x_max);
    // passx.setFilterLimits (-0.5, 0.5);

    // passx.setFilterLimits (-3.0, 3.0);
    //pass.setFilterLimitsNegative (true);
    passx.filter (*cloud_filtered);
    ROS_INFO("X filtered. %d", cloud_filtered->size());
  //****************************************************//



  //****************************************************//
    // // segment ground
    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // // Create the segmentation object
    // pcl::SACSegmentation<PointT> seg;
    // // Optional
    // seg.setOptimizeCoefficients (true);
    // // Mandatory
    // seg.setModelType (pcl::SACMODEL_PLANE);  // plane
    // seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setDistanceThreshold (0.010);

    // seg.setInputCloud (cloud_filtered);
    // seg.segment (*inliers, *coefficients);

    // pcl::ExtractIndices<PointT> extract;
    // extract.setInputCloud(cloud_filtered);
    // extract.setIndices(inliers);
    // extract.setNegative(true);
    // extract.filter(*cloud_filtered);
  //****************************************************//


  //****************************************************//
    // Create the filtering object - StatisticalOutlierRemoval filter
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud (cloud_filtered);
    sor.setMeanK (50);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);
    ROS_INFO("Outliers filtered. %d", cloud_filtered->size());

  //****************************************************//

    // pcl::PointCloud<PointT>::Ptr cloud_write (new pcl::PointCloud<PointT>);
    // cloud_write.width = cloud_filtered.points.size();
    // cloud_write.height = 1;
    // cloud_write.is_dense = false;

    // if(pcd_index == 0) {
    //   for (size_t i = 0; i < cloud_filtered->points.size(); i += 10)
    //     {
    //         ROS_INFO("X: %f Y: %f Z: %f", cloud_filtered->points[i].x, cloud_filtered->points[i].y, cloud_filtered->points[i].z);
    //     }
    // }

    return cloud_filtered;

}
