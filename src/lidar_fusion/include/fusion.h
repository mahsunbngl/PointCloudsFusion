#ifndef FUSION_INCLUDED
#define FUSION_INCLUDED

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/Bool.h"


class Fusion
{
    private:
        ros::NodeHandle* n;
        ros::Subscriber lidarSubscriber1;  
        ros::Subscriber lidarSubscriber2;   

        ros::Publisher fusedPointCloud; 

        sensor_msgs::PointCloud2 first_cloud;
        sensor_msgs::PointCloud2 second_cloud;
        sensor_msgs::PointCloud2 fused_cloud;



    public:
        Fusion (int argc , char **argv);
        
        void first_pc_callback(const sensor_msgs::PointCloud2ConstPtr& );
        void second_pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
        pcl::PointCloud<pcl::PointXYZ>::Ptr sensor2PCLConversion (const sensor_msgs::PointCloud2ConstPtr& );
        sensor_msgs::PointCloud2 fusionPointCLouds(const sensor_msgs::PointCloud2 &first, const sensor_msgs::PointCloud2 &second );

        // --------------------------- Transfrom ------------------------------------------------
        void transfrom (const sensor_msgs::PointCloud2ConstPtr& cloud_1 , const sensor_msgs::PointCloud2ConstPtr& cloud_2);


};


#endif