#ifndef DETECTION_INCLUDED
#define DETECTION_INCLUDED

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/Bool.h"
#include <geometry_msgs/Pose.h>
#include <pcl/segmentation/extract_clusters.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/filters/passthrough.h>


class Detection
{
    private:
        ros::NodeHandle* n;
        ros::Subscriber lidarSubscriber;   

        ros::Publisher cloud_filtered_pub;
        ros::Publisher pose_array_pub;
        ros::Publisher marker_array_pub;

        sensor_msgs::PointCloud2 point_cloud;
        sensor_msgs::PointCloud2 ros_pc2_out;        

        geometry_msgs::Pose pose;

        std::string sensor_model;

        bool print_fps;
        double z_axis_min;
        double z_axis_max;
        int cluster_size_min;
        int cluster_size_max;

        int regions[100];

        int frames; 
        clock_t start_time; 
        bool reset = true;//fps

        float tolerance = 0.0;




    public:
        Detection (int argc , char **argv);
        
        void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2_in);


};


#endif