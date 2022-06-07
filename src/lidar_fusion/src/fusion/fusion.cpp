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



    /*** Parameters ***/ 
    n->param <std::string> ("fcr_pc" , fcr_pc , "/carla/ego_vehicle/FCR");
    n->param <std::string> ("flr_pc" , flr_pc , "/carla/ego_vehicle/FLR");
    n->param <std::string> ("frr_pc" , frr_pc , "/carla/ego_vehicle/FRR");
    n->param <std::string> ("slr_pc" , slr_pc , "/carla/ego_vehicle/SLR");
    n->param <std::string> ("srr_pc" , srr_pc , "/carla/ego_vehicle/SRR");
    n->param <std::string> ("rcr_pc" , rcr_pc , "/carla/ego_vehicle/RCR");
    n->param <std::string> ("rlr_pc" , rlr_pc , "/carla/ego_vehicle/RLR");
    n->param <std::string> ("rrr_pc" , rrr_pc , "/carla/ego_vehicle/RRR");

    /*** Subscribers ***/
    radar_fcr_sub = n->subscribe(fcr_pc.c_str(), 10, &Fusion::fcr_pc_callback, this);
    radar_flr_sub = n->subscribe(flr_pc.c_str(), 10, &Fusion::flr_pc_callback, this);
    radar_frr_sub = n->subscribe(frr_pc.c_str(), 10, &Fusion::frr_pc_callback, this);
    radar_slr_sub = n->subscribe(slr_pc.c_str(), 10, &Fusion::slr_pc_callback, this);
    radar_srr_sub = n->subscribe(srr_pc.c_str(), 10, &Fusion::srr_pc_callback, this);
    radar_rcr_sub = n->subscribe(rcr_pc.c_str(), 10, &Fusion::rcr_pc_callback, this);
    radar_rlr_sub = n->subscribe(rlr_pc.c_str(), 10, &Fusion::rlr_pc_callback, this);
    radar_rrr_sub = n->subscribe(rrr_pc.c_str(), 10, &Fusion::rrr_pc_callback, this);


    /*** Publishers ***/
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


void Fusion::fcr_pc_callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
    // pcl::PointCloud<pcl::PointXYZ>::Ptr fcr_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // fcr_cloud_pc = sensor2PCLConversion (cloudMsg);
    fcr_cloud = *cloudMsg;

}

void Fusion::flr_pc_callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
    // pcl::PointCloud<pcl::PointXYZ>::Ptr flr_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // flr_cloud_pc = sensor2PCLConversion (cloudMsg);

    flr_cloud = *cloudMsg;
    fused_cloud = fusionPointCLouds (fcr_cloud,flr_cloud);
  
}

void Fusion::frr_pc_callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
    // pcl::PointCloud<pcl::PointXYZ>::Ptr frr_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // frr_cloud_pc = sensor2PCLConversion (cloudMsg);

    frr_cloud = *cloudMsg;

    fused_cloud = fusionPointCLouds (fused_cloud , frr_cloud);

}


void Fusion::slr_pc_callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
    // pcl::PointCloud<pcl::PointXYZ>::Ptr slr_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // slr_cloud_pc = sensor2PCLConversion (cloudMsg);

    slr_cloud = *cloudMsg;

    fused_cloud = fusionPointCLouds (fused_cloud , slr_cloud);
}
void Fusion::srr_pc_callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
    // pcl::PointCloud<pcl::PointXYZ>::Ptr srr_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // srr_cloud_pc = sensor2PCLConversion (cloudMsg);

    srr_cloud = *cloudMsg;

    fused_cloud = fusionPointCLouds (fused_cloud , srr_cloud);
}

void Fusion::rcr_pc_callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
    // pcl::PointCloud<pcl::PointXYZ>::Ptr rcr_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // rcr_cloud_pc = sensor2PCLConversion (cloudMsg);

    rcr_cloud = *cloudMsg;

    fused_cloud = fusionPointCLouds (fused_cloud , rcr_cloud);
}

void Fusion::rlr_pc_callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
    // pcl::PointCloud<pcl::PointXYZ>::Ptr rlr_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // rlr_cloud_pc = sensor2PCLConversion (cloudMsg);

    rlr_cloud = *cloudMsg;

    fused_cloud = fusionPointCLouds (fused_cloud , rlr_cloud);
}

void Fusion::rrr_pc_callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
    // pcl::PointCloud<pcl::PointXYZ>::Ptr rrr_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // rrr_cloud_pc = sensor2PCLConversion (cloudMsg);

    rrr_cloud = *cloudMsg;

    // fused_cloud = fusionPointCLouds (fused_cloud , rrr_cloud);
}


sensor_msgs::PointCloud2 Fusion::fusionPointCLouds(const sensor_msgs::PointCloud2& first, const sensor_msgs::PointCloud2 &second )
{
    


    pcl::concatenatePointCloud(first , second , result);

    std::cout << "Fused: " <<  pcl::concatenatePointCloud(first , second , result)<< "\n";

   

    // result.header.frame_id = "ego_vehicle";
    // result.header.stamp = ros::Time::now();

    fusedPointCloud.publish(result);

    return result;
}

