#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Point.h>

#include <detection.h>
#include <iostream>


int main(int argc, char **argv)
{

    Detection detection (argc, argv); 

    return 0;
}
