/**
 * 
 * I even don't remember when I moved from this file to  rotate_and_scan.cpp But it still here. Just in case.
 * 
 **/

#include <iostream>
#include <string>
#include <sstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int64.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <Eigen/Eigen>

// Get point cloud, filter it by coordinates and publish it

typedef pcl::PointXYZRGBA PointT;
static int pcd_index = 0;
static char gotDataFlag = 0;// could use a 'class' to reduce this global variable
static ros::Publisher pcl_pub;
static double x_min = -0.5;
static double x_max = 0.5;
static double y_min = -0.5;
static double y_max = 0.5;
static double z_min = -0.5;
static double z_max = 0.5;

pcl::PointCloud<PointT>::Ptr
cloud_filter(pcl::PointCloud<PointT>::Ptr &cloud);

void callback(sensor_msgs::PointCloud2 cloud_raw)
{
    // cloud_raw is PC data from Kinect V2;
    // static int pcd_index = 0;
    pcl::PointCloud<PointT>::Ptr cloud_ptr (new pcl::PointCloud<PointT>);
    std::string filename = "/home/k3dr/catkin_ws/src/scanner_pcl/data/" + std::to_string(pcd_index) + ".pcd";

    ROS_INFO("Processing #%i PointCloud...", pcd_index);

    // change PC format from PointCloud2 to pcl::PointCloud<PointT>
    pcl::fromROSMsg(cloud_raw, *cloud_ptr);

    // crop, segment, filter
    cloud_ptr = cloud_filter(cloud_ptr);

    // save PCD file to local folder
    // TODO am I really need to store it on drive?
    if(pcd_index == 0) {
        pcl::io::savePCDFileBinary (filename, *cloud_ptr);
    }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_ptr, output);
    ROS_INFO("Processed size: %d", cloud_ptr->size());
    pcl_pub.publish(output);

    gotDataFlag = 1;
    ++pcd_index;
}


int main (int argc, char **argv)
{
    ros::init (argc, argv, "pcl_processing");

    if( argc > 1 ) {
       x_min = atof(argv[1]);  // alternative strtod
       x_max = atof(argv[2]);
       y_min = atof(argv[3]);
       y_max = atof(argv[4]);
       z_min = atof(argv[5]);
       z_max = atof(argv[6]);
    }

    ros::NodeHandle nh; 
    ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1 , callback);
    ros::Publisher pub = nh.advertise<std_msgs::Int64> ("pcd_save_done", 1);
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered", 1);

    ros::Rate loop_rate(0.1);
    std_msgs::Int64 number_PCDdone;
    // std::stringstream ss;
    while (ros::ok())
    {
        /* Do something? */
        // ss.str("");
        // ss << "have saved pcd #" << pcd_index ;
        // msg.data = ss.str();
        number_PCDdone.data = pcd_index;

        // ros::spin()
        //*** only when this is run, it will get to callback
        

        // only publish data when having got data
        if (gotDataFlag == 1){
            pub.publish(number_PCDdone);
            gotDataFlag = 0;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


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

    if(pcd_index == 0) {
      for (size_t i = 0; i < cloud_filtered->points.size(); i += 10)
        {
            ROS_INFO("X: %f Y: %f Z: %f", cloud_filtered->points[i].x, cloud_filtered->points[i].y, cloud_filtered->points[i].z);
        }
    }

    return cloud_filtered;

}