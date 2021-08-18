#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>


// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Point.h>
#include "std_msgs/Bool.h"


#include <iostream>
#include <string>

#include "transform.h"


Transform::Transform (int argc , char **argv)
{
    ros::init(argc, argv , "Point_Cloud_Transfrom");
    n = new ros::NodeHandle("~");


    std::string first_lidar_topic; 
    std::string second_lidar_topic; 

    pcl::PointCloud<pcl::PointXYZ>::Ptr first_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr second_cloud;


    n->param <std::string> ("first_lidar_topic" , first_lidar_topic , "/velodyne_points");
    n->param <std::string> ("second_lidar_topic" , second_lidar_topic , "/velodyne_points2");

    n->param <double>("lidar_pos_x", lidar_pos_x, 0.0);
    n->param <double>("lidar_pos_y", lidar_pos_y, 0.0);
    n->param <double>("lidar_pos_z", lidar_pos_z, 0.2);
    n->param <double>("lidar_rotation", lidar_rotation, 0.0);

    lidarSubscriber1 = n->subscribe(first_lidar_topic.c_str(), 10, &Transform::first_pc_callback, this);
    lidarSubscriber2 = n->subscribe(second_lidar_topic.c_str(), 10, &Transform::second_pc_callback, this);

    transformedCloudPub = n->advertise<sensor_msgs::PointCloud2> ("/transformed", 10);

    ros::spin();    
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Transform::sensor2PCLConversion (const sensor_msgs::PointCloud2ConstPtr& cloudMsg ) // Transform sensor_msgs::PointCloud2::ConstPtr to a pcl::PointCloud<pcl::pointxyz>::Ptr
{
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl_conversions::toPCL(*cloudMsg, *cloud);                  
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud,*xyz_cloud);

    return xyz_cloud;
}


void Transform::first_pc_callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{

    first_cloud = sensor2PCLConversion (cloudMsg);

    firstcloudmsg = *cloudMsg;

    std::cout << "firstcloudmsg.fields.size() = " << firstcloudmsg.fields.size() << "\n";

    transform(cloudMsg , lidar_pos_x , lidar_pos_y, lidar_pos_z , lidar_rotation);
}

void Transform::second_pc_callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
    second_cloud = sensor2PCLConversion (cloudMsg);

    secondcloudmsg = *cloudMsg;

}

void Transform::transform(const sensor_msgs::PointCloud2ConstPtr& cloudMsg , double x , double y , double z , double rotation_angle)
{

    
    /* Reminder: how transformation matrices work :

           |-------> This column is the translation
    | 1 0 0 x |  \
    | 0 1 0 y |   }-> The identity 3x3 matrix (no rotation) on the left
    | 0 0 1 z |  /
    | 0 0 0 1 |    -> We do not use this line (and it has to stay 0,0,0,1)

    METHOD #1: Using a Matrix4f
    This is the "manual" method, perfect to understand but error prone !
    */
    

    // Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
    theta = (M_PI/180)*lidar_rotation; // The angle of rotation in degree
    transform_1 (0,0) = std::cos (theta);
    transform_1 (0,1) = -sin(theta);
    transform_1 (1,0) = sin (theta);
    transform_1 (1,1) = std::cos (theta);
    //    (row, column)

    // Define a translation
    transform_1 (0,3) = lidar_pos_x;
    transform_1 (1,3) = lidar_pos_y;
    transform_1 (2,3) = lidar_pos_z;

    // Print the transformation
    printf ("Method #1: using a Matrix4f\n");
    std::cout << transform_1 << std::endl;

    /*  METHOD #2: Using a Affine3f
        This method is easier and less error prone
    */
    
    // Define a translation of 2.5 meters on the x axis.
    // transform_2.translation() << 0, 0, 0.2;

    // The same rotation matrix as before; theta radians around Z axis
    // transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

    // Print the transformation
    // printf ("\nMethod #2: using an Affine3f\n");
    // std::cout << transform_2.matrix() << std::endl;

    // You can either apply transform_1 or transform_2; they are the same
    // pcl::transformPointCloud (*pc1, *transformed_cloud, transform_1, true);

    pcl_ros::transformPointCloud(transform_1 , firstcloudmsg , transformedcloudmsg );

    transformedCloudPub.publish(transformedcloudmsg);
}





