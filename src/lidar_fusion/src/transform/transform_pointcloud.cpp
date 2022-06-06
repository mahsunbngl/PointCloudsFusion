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
    ros::init(argc, argv , "Point_Cloud_Transform");
    n = new ros::NodeHandle("~");


    std::string flr_topic; 
    std::string frr_topic; 
    std::string slr_topic;
    std::string srr_topic;
    std::string rcr_topic;
    std::string rlr_topic;
    std::string rrr_topic;


    /*** Parameters ***/ 
    n->param <std::string> ("flr_topic" , flr_topic , "/carla/ego_vehicle/FLR");
    n->param <std::string> ("frr_topic" , frr_topic , "/carla/ego_vehicle/FRR");
    n->param <std::string> ("slr_topic" , slr_topic , "/carla/ego_vehicle/SLR");
    n->param <std::string> ("srr_topic" , srr_topic , "/carla/ego_vehicle/SRR");
    n->param <std::string> ("rcr_topic" , rcr_topic , "/carla/ego_vehicle/RCR");
    n->param <std::string> ("rlr_topic" , rlr_topic , "/carla/ego_vehicle/RLR");
    n->param <std::string> ("rcr_topic" , rcr_topic , "/carla/ego_vehicle/RRR");

    // n->param <double>("lidar_pos_x", lidar_pos_x, 0.0);
    // n->param <double>("lidar_pos_y", lidar_pos_y, 0.0);
    // n->param <double>("lidar_pos_z", lidar_pos_z, -0.2);
    // n->param <double>("lidar_rotation", lidar_rotation, 0.0);

    /*** Subscribers ***/
    lidarSubscriber1 = n->subscribe(flr_topic.c_str(), 10, &Transform::FLR_pc_callback, this);
    lidarSubscriber2 = n->subscribe(frr_topic.c_str(), 10, &Transform::FRR_pc_callback, this);
    lidarSubscriber3 = n->subscribe(slr_topic.c_str(), 10, &Transform::SLR_pc_callback, this);
    lidarSubscriber4 = n->subscribe(srr_topic.c_str(), 10, &Transform::SRR_pc_callback, this);
    lidarSubscriber5 = n->subscribe(rcr_topic.c_str(), 10, &Transform::RCR_pc_callback, this);
    lidarSubscriber6 = n->subscribe(rlr_topic.c_str(), 10, &Transform::RLR_pc_callback, this);
    lidarSubscriber7 = n->subscribe(rcr_topic.c_str(), 10, &Transform::RRR_pc_callback, this);



    /*** Publishers ***/
    FLRtransformedCloudPub = n->advertise<sensor_msgs::PointCloud2> ("/transformed_FLR", 10);
    FRRtransformedCloudPub = n->advertise<sensor_msgs::PointCloud2> ("/transformed_FRR", 10);
    SLRtransformedCloudPub = n->advertise<sensor_msgs::PointCloud2> ("/transformed_SLR", 10);
    SRRtransformedCloudPub = n->advertise<sensor_msgs::PointCloud2> ("/transformed_SRR", 10);
    RCRtransformedCloudPub = n->advertise<sensor_msgs::PointCloud2> ("/transformed_RCR", 10);
    RLRtransformedCloudPub = n->advertise<sensor_msgs::PointCloud2> ("/transformed_RLR", 10);
    RRRtransformedCloudPub = n->advertise<sensor_msgs::PointCloud2> ("/transformed_RRR", 10);

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


void Transform::FLR_pc_callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{

    // first_cloud = sensor2PCLConversion (cloudMsg);

    transform(cloudMsg , 0 , 0, 0 , 0, FLRtransformedcloudmsg, FLRtransformedCloudPub);
}

void Transform::FRR_pc_callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{

    transform(cloudMsg , 0 , 0, 0 , 0, FRRtransformedcloudmsg, FRRtransformedCloudPub);
}

void Transform::SLR_pc_callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
   
    transform(cloudMsg , 0 , 0, 0, 0, SLRtransformedcloudmsg, SLRtransformedCloudPub);
}
void Transform::SRR_pc_callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
    transform(cloudMsg , 0 , 0, 0 , 0, SRRtransformedcloudmsg, SRRtransformedCloudPub);

}
void Transform::RCR_pc_callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
    transform(cloudMsg , 0 , 0, 0 , 0, RCRtransformedcloudmsg, RCRtransformedCloudPub);

}
void Transform::RLR_pc_callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
    transform(cloudMsg , 0 , 0, 0 , 0, RLRtransformedcloudmsg, RLRtransformedCloudPub);

}
void Transform::RRR_pc_callback(const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
    transform(cloudMsg , 0 , 0, 0 , 0, RRRtransformedcloudmsg, RRRtransformedCloudPub);

}

void Transform::transform(const sensor_msgs::PointCloud2ConstPtr& cloudMsg , double x , double y , double z , 
                            double rotation_angle, sensor_msgs::PointCloud2 transformedcloudmsg, ros::Publisher radar_transformed_pub)
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
    theta = (M_PI/180)*rotation_angle; // The angle of rotation in degree
    transform_1 (0,0) = std::cos (theta);
    transform_1 (0,1) = -sin(theta);
    transform_1 (1,0) = sin (theta);
    transform_1 (1,1) = std::cos (theta);
    //    (row, column)

    // Define a translation
    transform_1 (0,3) = x;
    transform_1 (1,3) = y;
    transform_1 (2,3) = z;

    // Print the transformation
    std::cout << "Method #1: using a Matrix4f\n";
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

    pcl_ros::transformPointCloud(transform_1 , *cloudMsg , transformedcloudmsg );

    radar_transformed_pub.publish(transformedcloudmsg);
}





