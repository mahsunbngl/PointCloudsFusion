## Object Detection with Multiple Lidar Fusion
The aim of this project is to concatenate two point cloud with rigid body transform and detect objects by using PCL (Point Cloud Library). The project includes;

- Taking two different point clouds with different axises and transform one of them in order to make their z axis the same. The algorithm publishes the transformed point cloud data in `/transformed` topic.

    `rosrun lidar_fusion transform_pointcloud_main`
    
 - Taking the transformed point cloud for concatenating it with the second point cloud. The algorithm publishes fused point cloud data in `/fused` topic.  

    `rosrun lidar_fusion fusion_main`
    
 - Taking the fused point cloud in order to detect objects. The algorithm publishes found objects poses in `/ObjectDetection/poses`  , clustered point clouds in `/ObjectDetection/cloud_filtered` and markers of the objects in `/ObjectDetection/markers` topics.
 
    `rosrun lidar_fusion detection_main`
    
 ### Parameters
 
**transform_pointcloud**
| **Parameters** |Definition| 
|-----|----|
| **first_lidar_topic** |The first point cloud topic.| 
| **second_lidar_topic** |The second point cloud topic. This point cloud gets transformed but if required both point clouds can get transformed.|
| **lidar_pos_x** |Desired value of transformation on x axis.| 
| **lidar_pos_y** |Desired value of transformation on y axis.| 
| **lidar_pos_z** |Desired value of transformation on z axis.| 
| **lidar_rotation** |Desired value of transformation on yaw.| 

**fusion**
| **Parameters** |Definition| 
|-----|----|
| **first_pc** |The first point cloud topic.| 
| **second_pc** |The second point cloud topic.|

**detection**
| **Parameters** |Definition| 
|-----|----|
| **sensor_model** |Sensor model. (In this project Velodyne HDL-323 is used.)| 
| **z_axis_min** |Desired value of minimum z axis of region of interest for removing the ground.|
| **z_axis_max** |Desired value of maximum z axis of region of interest for removing the ceiling.|
| **cluster_size_min** |The minimum number of points that a cluster needs to contain in order to be considered valid.|
| **cluster_size_max** |The maximum number of points that a cluster needs to contain in order to be considered valid.|
| **lidar_topic** |The input point cloud topic. |

### Install
This package has three main dependencies:
 
  `sudo apt install ros-$ROS_DISTRO-velodyne-simulator`
  
  `sudo apt install ros-$ROS_DISTRO-perception-pcl`
  
  `sudo apt install ros-$ROS_DISTRO-pcl-conversions`
  
ROSDep will take care of the other dependencies.

  `rosdep install --from-paths src --ignore-src -r -y`
  
You can run via `roslaunch lidar_fusion overall.launch`


