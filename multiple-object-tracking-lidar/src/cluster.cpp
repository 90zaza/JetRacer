#include "kf_tracker/CKalmanFilter.h"
#include "kf_tracker/featureDetection.h"
#include "opencv2/video/tracking.hpp"
#include "pcl_ros/point_cloud.h"
#include <algorithm>
#include <fstream>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <iterator>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <string.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/geometry.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <limits>
#include <utility>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

ros::Publisher pub_filtered_cloud;

void publish_cloud(ros::Publisher &pub, pcl::PointCloud<pcl::PointXYZ>::Ptr cluster) {
  sensor_msgs::PointCloud2::Ptr clustermsg(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(*cluster, *clustermsg);
  clustermsg->header.frame_id = "map";
  clustermsg->header.stamp = ros::Time::now();
  pub.publish(*clustermsg);
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)

{
  // Process the point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr clustered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  /* Creating the KdTree from input point cloud*/
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

  pcl::fromROSMsg(*input, *input_cloud);
  tree->setInputCloud(input_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(0.1);
  ec.setMinClusterSize(15);
  ec.setMaxClusterSize(1000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(input_cloud);
  /* Extract the clusters out of pc and save indices in cluster_indices.*/
  ec.extract(cluster_indices);
  std::vector<pcl::PointIndices>::const_iterator it;
  std::vector<int>::const_iterator pit;
  
  for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    int numPts = 0;
    int minIndex, maxIndex;
    double minY = 1.5;
    double maxY = -1.5;
    for (pit = it->indices.begin(); pit != it->indices.end(); pit++) {
      cloud_cluster->points.push_back(input_cloud->points[*pit]);
      numPts += 1;
      if (input_cloud->points[*pit].y < minY) {
        minY = input_cloud->points[*pit].y;
        minIndex = numPts - 1;
      }
      if (input_cloud->points[*pit].y > maxY) {
        maxY = input_cloud->points[*pit].y;
        maxIndex = numPts - 1;
      }
    }
     
    //std::cout << "y: " << minY << " " << maxY << "\n";
 
    pcl::PointXYZ minPt, maxPt;
    minPt = cloud_cluster->points[minIndex];
    maxPt = cloud_cluster->points[maxIndex];

    //std::cout << "y: " << minPt.y << " " << maxPt.y << "\n";

    double length = sqrt(pow((maxPt.y - minPt.y), 2) + pow((maxPt.x - minPt.x), 2));

    //std::cout << length << "\n";

    if (length < 0.25 && length > 0.15) {
    *clustered_cloud+= *cloud_cluster;
    }
  }

  publish_cloud(pub_filtered_cloud, clustered_cloud);
}
  
int main(int argc, char **argv) {
  // ROS init
  ros::init(argc, argv, "cluster");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("PCLcar1", 1, cloud_cb);
  pub_filtered_cloud = nh.advertise<sensor_msgs::PointCloud2>("PCLfilteredCar1", 1);

  ros::spin();
}
