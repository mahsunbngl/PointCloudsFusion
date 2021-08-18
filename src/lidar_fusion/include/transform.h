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

        ros::Publisher transformedCloudPub; 

        sensor_msgs::PointCloud2 transformedcloudmsg;
        sensor_msgs::PointCloud2 firstcloudmsg;
        sensor_msgs::PointCloud2 secondcloudmsg;

        pcl::PointCloud<pcl::PointXYZ>::Ptr first_cloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr second_cloud;            

        Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
        Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

        double lidar_pos_x;
        double lidar_pos_y;
        double lidar_pos_z;
        double lidar_rotation;

        double theta;



    public:
        Transform (int argc , char **argv);
        
        void first_pc_callback(const sensor_msgs::PointCloud2ConstPtr& );
        void second_pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
        pcl::PointCloud<pcl::PointXYZ>::Ptr sensor2PCLConversion (const sensor_msgs::PointCloud2ConstPtr& );

        void transform(const sensor_msgs::PointCloud2ConstPtr& cloudMsg ,double , double , double ,double);


};


#endif