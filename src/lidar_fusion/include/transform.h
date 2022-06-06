#ifndef TRANSFORM_INCLUDED
#define TRANSFORM_INCLUDED

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/Bool.h"


class Transform
{
    private:
        ros::NodeHandle* n;
        ros::Subscriber lidarSubscriber1;  
        ros::Subscriber lidarSubscriber2;   
        ros::Subscriber lidarSubscriber3; 
        ros::Subscriber lidarSubscriber4; 
        ros::Subscriber lidarSubscriber5; 
        ros::Subscriber lidarSubscriber6; 
        ros::Subscriber lidarSubscriber7; 


        ros::Publisher FLRtransformedCloudPub; 
        ros::Publisher FRRtransformedCloudPub;
        ros::Publisher SLRtransformedCloudPub;
        ros::Publisher SRRtransformedCloudPub;
        ros::Publisher RCRtransformedCloudPub;
        ros::Publisher RLRtransformedCloudPub;
        ros::Publisher RRRtransformedCloudPub;

        sensor_msgs::PointCloud2 FLRtransformedcloudmsg;
        sensor_msgs::PointCloud2 FRRtransformedcloudmsg;
        sensor_msgs::PointCloud2 SLRtransformedcloudmsg;
        sensor_msgs::PointCloud2 SRRtransformedcloudmsg;
        sensor_msgs::PointCloud2 RCRtransformedcloudmsg;
        sensor_msgs::PointCloud2 RLRtransformedcloudmsg;
        sensor_msgs::PointCloud2 RRRtransformedcloudmsg;


        pcl::PointCloud<pcl::PointXYZ>::Ptr first_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr second_cloud;  
        pcl::PointCloud<pcl::PointXYZ>::Ptr third_cloud;
          

        Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
        Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();
        Eigen::Affine3f transform_4 = Eigen::Affine3f::Identity();

        double lidar_pos_x;
        double lidar_pos_y;
        double lidar_pos_z;
        double lidar_rotation;

        double theta;



    public:
        Transform (int argc , char **argv);
        
        void FLR_pc_callback(const sensor_msgs::PointCloud2ConstPtr& );
        void FRR_pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
        void SLR_pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
        void SRR_pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
        void RCR_pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
        void RLR_pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
        void RRR_pc_callback(const sensor_msgs::PointCloud2ConstPtr&);

        pcl::PointCloud<pcl::PointXYZ>::Ptr sensor2PCLConversion (const sensor_msgs::PointCloud2ConstPtr& );

        void transform(const sensor_msgs::PointCloud2ConstPtr& cloudMsg ,double , double , double ,double, sensor_msgs::PointCloud2,ros::Publisher);



};


#endif