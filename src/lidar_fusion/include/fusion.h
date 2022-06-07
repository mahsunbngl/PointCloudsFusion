#ifndef FUSION_INCLUDED
#define FUSION_INCLUDED

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/Bool.h"


class Fusion
{
    private:
        ros::NodeHandle* n;
        ros::Subscriber radar_fcr_sub;  
        ros::Subscriber radar_flr_sub;   
        ros::Subscriber radar_frr_sub;
        ros::Subscriber radar_slr_sub;
        ros::Subscriber radar_srr_sub;  
        ros::Subscriber radar_rcr_sub;   
        ros::Subscriber radar_rlr_sub;
        ros::Subscriber radar_rrr_sub;

        ros::Publisher fusedPointCloud; 

        sensor_msgs::PointCloud2 fcr_cloud;
        sensor_msgs::PointCloud2 flr_cloud;
        sensor_msgs::PointCloud2 frr_cloud;
        sensor_msgs::PointCloud2 slr_cloud;
        sensor_msgs::PointCloud2 srr_cloud;
        sensor_msgs::PointCloud2 rcr_cloud;
        sensor_msgs::PointCloud2 rlr_cloud;
        sensor_msgs::PointCloud2 rrr_cloud;
        sensor_msgs::PointCloud2 fused_cloud;

        sensor_msgs::PointCloud2 result;

        std::string fcr_pc; 
        std::string flr_pc; 
        std::string frr_pc; 
        std::string slr_pc; 
        std::string srr_pc; 
        std::string rcr_pc; 
        std::string rlr_pc; 
        std::string rrr_pc; 

        pcl::PointCloud<pcl::PointXYZ>::Ptr fcr_cloud_pc;
        pcl::PointCloud<pcl::PointXYZ>::Ptr flr_cloud_pc;
        pcl::PointCloud<pcl::PointXYZ>::Ptr frr_cloud_pc;
        pcl::PointCloud<pcl::PointXYZ>::Ptr slr_cloud_pc;
        pcl::PointCloud<pcl::PointXYZ>::Ptr srr_cloud_pc;
        pcl::PointCloud<pcl::PointXYZ>::Ptr rcr_cloud_pc;
        pcl::PointCloud<pcl::PointXYZ>::Ptr rlr_cloud_pc;
        pcl::PointCloud<pcl::PointXYZ>::Ptr rrr_cloud_pc;



    public:
        Fusion (int argc , char **argv);
        
        void fcr_pc_callback(const sensor_msgs::PointCloud2ConstPtr& );
        void flr_pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
        void frr_pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
        void slr_pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
        void srr_pc_callback(const sensor_msgs::PointCloud2ConstPtr& );
        void rcr_pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
        void rlr_pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
        void rrr_pc_callback(const sensor_msgs::PointCloud2ConstPtr&);
        pcl::PointCloud<pcl::PointXYZ>::Ptr sensor2PCLConversion (const sensor_msgs::PointCloud2ConstPtr& );
        sensor_msgs::PointCloud2 fusionPointCLouds(const sensor_msgs::PointCloud2 &first, const sensor_msgs::PointCloud2 &second );


};


#endif