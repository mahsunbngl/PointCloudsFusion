#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>

#include <geometry_msgs/Point.h>
#include "std_msgs/Bool.h"


#include <iostream>
#include <string>

#include "fusion.h"


Fusion::Fusion (int argc , char **argv)
{
    ros::init(argc, argv , "Point_Cloud_Fusion");
    n = new ros::NodeHandle("~");


    std::string first_lidar_topic; 
    std::string second_lidar_topic; 

    pcl::PointCloud<pcl::PointXYZ>::Ptr first_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr second_cloud;

    n->param <std::string> ("first_lidar_topic" , first_lidar_topic , "/transformed");
    n->param <std::string> ("second_lidar_topic" , second_lidar_topic , "/velodyne_points2");

    lidarSubscriber1 = n->subscribe(first_lidar_topic.c_str(), 10, &Fusion::first_pc_callback, this);
    lidarSubscriber2 = n->subscribe(second_lidar_topic.c_str(), 10, &Fusion::second_pc_callback, this);

    fusedPointCloud = n->advertise<sensor_msgs::PointCloud2> ("/fused", 10); 


    ros::spin();    
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Fusion::sensor2PCLConversion (const sensor_msgs::PointCloud2ConstPtr& cloudMsg ) // Transform sensor_msgs::PointCloud2::ConstPtr to a pcl::PointCloud<pcl::pointxyz>::Ptr
{
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*cloudMsg, *cloud);                  
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud,*xyz_cloud);

    return xyz_cloud;
}

void Fusion::first_pc_callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
    // pcl::PointCloud<pcl::PointXYZ>::Ptr first_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // first_cloud = sensor2PCLConversion (cloudMsg);
    first_cloud = *cloudMsg;

}

void Fusion::second_pc_callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
    // pcl::PointCloud<pcl::PointXYZ>::Ptr second_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // second_cloud = sensor2PCLConversion (cloudMsg);

    second_cloud = *cloudMsg;

    fused_cloud = fusionPointCLouds (first_cloud , second_cloud);
}

sensor_msgs::PointCloud2 Fusion::fusionPointCLouds(const sensor_msgs::PointCloud2& first_cloud, const sensor_msgs::PointCloud2 &second_cloud )
{
    sensor_msgs::PointCloud2 result;


    pcl::concatenatePointCloud(first_cloud , second_cloud , result);

    std::cout << "Fused: " <<  pcl::concatenatePointCloud(first_cloud , second_cloud , result)<< "\n";

    fusedPointCloud.publish(result);

    return result;
}