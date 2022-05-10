#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include "adaptive_clustering/ClusterArray.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <iostream>

#include "detection.h"


Detection::Detection (int argc , char **argv)
{
  ros::init(argc, argv, "Detection");
  n = new ros::NodeHandle("~");


  std::string lidar_topic; 

  /*** Parameters ***/ 
  n->param<std::string>("sensor_model", sensor_model, "VLP-16"); // VLP-16, HDL-32E, HDL-64E
  n->param<double>("z_axis_min", z_axis_min, 0);
  n->param<double>("z_axis_max", z_axis_max, 0.5);
  n->param<int>("cluster_size_min", cluster_size_min, 2);
  n->param<int>("cluster_size_max", cluster_size_max, 20);
  n->param <std::string> ("lidar_topic" , lidar_topic , "/carla/ego_vehicle/radar_front");

  
  /*** Subscribers ***/
  lidarSubscriber = n->subscribe(lidar_topic.c_str(), 10, &Detection::pointCloudCallback, this);

  /*** Publishers ***/
  cloud_filtered_pub = n->advertise<sensor_msgs::PointCloud2>("cloud_filtered", 10);
  pose_array_pub = n->advertise<geometry_msgs::PoseArray>("poses", 10);
  marker_array_pub = n->advertise<visualization_msgs::MarkerArray>("markers", 10);
  
  ros::spin();
}

void Detection::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_pc2_in)
{
  const int region_max = 38; // How far you want to detect.

  // Divide the point cloud into nested circular regions centred at the sensor.
  // For more details, see our IROS-17 paper "Online learning for human classification in 3D LiDAR-based tracking"
  if(sensor_model.compare("VLP-16") == 0) {
    regions[0] = 4; regions[1] = 6; regions[2] = 6; regions[3] = 6; regions[4] = 6;
    regions[5] = 6; regions[6] = 6; regions[7] = 4; regions[8] = 6; regions[9] = 6;
    regions[10]= 6; regions[11]= 6; regions[12]= 6; regions[13]= 6;
  } else if (sensor_model.compare("HDL-32E") == 0) {
    regions[0] = 4; regions[1] = 5; regions[2] = 4; regions[3] = 5; regions[4] = 4;
    regions[5] = 5; regions[6] = 5; regions[7] = 4; regions[8] = 5; regions[9] = 4;
    regions[10]= 5; regions[11]= 5; regions[12]= 4; regions[13]= 5;
  } else if (sensor_model.compare("HDL-64E") == 0) {
    regions[0] = 14; regions[1] = 14; regions[2] = 14; regions[3] = 15; regions[4] = 14;
  } else {
    ROS_FATAL("Unknown sensor model!");
  }

  
  /*** Convert ROS message to PCL ***/
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*ros_pc2_in, *pcl_pc_in);
  
  /*** Remove ground and ceiling ***/
  pcl::IndicesPtr pc_indices(new std::vector<int>);
  pcl::PassThrough<pcl::PointXYZI> pt;
  pt.setInputCloud(pcl_pc_in);
  pt.setFilterFieldName("z");
  pt.setFilterLimits(z_axis_min, z_axis_max);
  pt.filter(*pc_indices);
  
  /*** Divide the point cloud into nested circular regions ***/
  boost::array<std::vector<int>, region_max> indices_array;
  for(int i = 0; i < pc_indices->size(); i++)
  {
    float range = 0.0;
    for(int j = 0; j < region_max; j++)
    {
      float d2 = pcl_pc_in->points[(*pc_indices)[i]].x * pcl_pc_in->points[(*pc_indices)[i]].x +
	    pcl_pc_in->points[(*pc_indices)[i]].y * pcl_pc_in->points[(*pc_indices)[i]].y +
	    pcl_pc_in->points[(*pc_indices)[i]].z * pcl_pc_in->points[(*pc_indices)[i]].z;

      if(d2 > range * range && d2 <= (range+regions[j]) * (range+regions[j]))
      {
      	indices_array[j].push_back((*pc_indices)[i]);
      	break;
      }
      range += regions[j];
    }
  }
  
  /*** Euclidean clustering ***/
  float tolerance = 0.0;
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr > > clusters;
  
  for(int i = 0; i < region_max; i++)
  {
    tolerance += 0.5;
    if(indices_array[i].size() > cluster_size_min) 
    {
      boost::shared_ptr<std::vector<int> > indices_array_ptr(new std::vector<int>(indices_array[i]));
      pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
      tree->setInputCloud(pcl_pc_in, indices_array_ptr);
      
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
      ec.setClusterTolerance(tolerance);
      ec.setMinClusterSize(cluster_size_min);
      ec.setMaxClusterSize(cluster_size_max);
      ec.setSearchMethod(tree);
      ec.setInputCloud(pcl_pc_in);
      ec.setIndices(indices_array_ptr);
      ec.extract(cluster_indices);
      
      for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); it++)
      {
      	pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
      	for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
      	  cluster->points.push_back(pcl_pc_in->points[*pit]);
  	    }
      	cluster->width = cluster->size();
      	cluster->height = 1;
      	cluster->is_dense = true;
	      clusters.push_back(cluster);

        
      }
    }
  }
  
  /*** Output ***/
  if(cloud_filtered_pub.getNumSubscribers() > 0)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_out(new pcl::PointCloud<pcl::PointXYZI>);
    sensor_msgs::PointCloud2 ros_pc2_out;
    pcl::copyPointCloud(*pcl_pc_in, *pc_indices, *pcl_pc_out);
    pcl::toROSMsg(*pcl_pc_out, ros_pc2_out);
    cloud_filtered_pub.publish(ros_pc2_out);
  }
  
  // adaptive_clustering::ClusterArray cluster_array;
  geometry_msgs::PoseArray pose_array;
  visualization_msgs::MarkerArray marker_array;
   

  for(int i = 0; i < clusters.size(); i++) 
  {
    // if(cluster_array_pub_.getNumSubscribers() > 0) {
    //   sensor_msgs::PointCloud2 ros_pc2_out;
    //   pcl::toROSMsg(*clusters[i], ros_pc2_out);
    //   cluster_array.clusters.push_back(ros_pc2_out);
      
    // }



    if(pose_array_pub.getNumSubscribers() > 0) 
    {
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*clusters[i], centroid);
      
      pose.position.x = centroid[0];
      pose.position.y = centroid[1];
      pose.position.z = centroid[2];
      pose.orientation.w = 1;
      pose_array.poses.push_back(pose);
    }
    
    if(marker_array_pub.getNumSubscribers() > 0) 
    {
      Eigen::Vector4f min, max;
      pcl::getMinMax3D(*clusters[i], min, max);
      
      visualization_msgs::Marker marker;
      marker.header = ros_pc2_in->header;
      marker.ns = "adaptive_clustering";
      marker.id = i;
      marker.type = visualization_msgs::Marker::LINE_LIST;

      
      geometry_msgs::Point p[24];
      p[0].x = max[0];  p[0].y = max[1];  p[0].z = max[2];
      p[1].x = min[0];  p[1].y = max[1];  p[1].z = max[2];
      p[2].x = max[0];  p[2].y = max[1];  p[2].z = max[2];
      p[3].x = max[0];  p[3].y = min[1];  p[3].z = max[2];
      p[4].x = max[0];  p[4].y = max[1];  p[4].z = max[2];
      p[5].x = max[0];  p[5].y = max[1];  p[5].z = min[2];
      p[6].x = min[0];  p[6].y = min[1];  p[6].z = min[2];
      p[7].x = max[0];  p[7].y = min[1];  p[7].z = min[2];
      p[8].x = min[0];  p[8].y = min[1];  p[8].z = min[2];
      p[9].x = min[0];  p[9].y = max[1];  p[9].z = min[2];
      p[10].x = min[0]; p[10].y = min[1]; p[10].z = min[2];
      p[11].x = min[0]; p[11].y = min[1]; p[11].z = max[2];
      p[12].x = min[0]; p[12].y = max[1]; p[12].z = max[2];
      p[13].x = min[0]; p[13].y = max[1]; p[13].z = min[2];
      p[14].x = min[0]; p[14].y = max[1]; p[14].z = max[2];
      p[15].x = min[0]; p[15].y = min[1]; p[15].z = max[2];
      p[16].x = max[0]; p[16].y = min[1]; p[16].z = max[2];
      p[17].x = max[0]; p[17].y = min[1]; p[17].z = min[2];
      p[18].x = max[0]; p[18].y = min[1]; p[18].z = max[2];
      p[19].x = min[0]; p[19].y = min[1]; p[19].z = max[2];
      p[20].x = max[0]; p[20].y = max[1]; p[20].z = min[2];
      p[21].x = min[0]; p[21].y = max[1]; p[21].z = min[2];
      p[22].x = max[0]; p[22].y = max[1]; p[22].z = min[2];
      p[23].x = max[0]; p[23].y = min[1]; p[23].z = min[2];

      for(int i = 0; i < 24; i++)
      {
  	    marker.points.push_back(p[i]);
      }
      
      marker.scale.x = 0.5;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.5;
      marker.lifetime = ros::Duration(0.1);
      marker_array.markers.push_back(marker);

    }
  }
  

  if(pose_array.poses.size())
  {
    pose_array.header = ros_pc2_in->header;
    pose_array_pub.publish(pose_array);
  }
  
  if(marker_array.markers.size())
  {
    marker_array_pub.publish(marker_array);
  }


}

