//#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <ros/ros.h>
#include <bits/stdc++.h>
#include <list>
#include <algorithm>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <visualization_msgs/MarkerArray.h>

#define pi 3.14159265359

//boost::shared_ptr<visualization_msgs::MarkerArray const> mainMSG;
std::list<double> object_list;
int objectNumberCar;
std::string objectTopicCar;
ros::Publisher pub_carInFront;

//int initialiser(boost::shared_ptr<visualization_msgs::MarkerArray const> incoming_array) {
//  for (int i = 0; i < 6; i++) {
//    if (incoming_array.get()->markers[i].pose.position.x > 0.15 && incoming_array.get()->markers[i].pose.position.x < 0.20){
//      object_list.push_back(i);
//      if (object_list.size() > 1 && object_list.size() != 0) {
//	if (incoming_array.get()->markers[i].pose.position.y == 0 && incoming_array.get()->markers[i].pose.position.y > 0.05 && incoming_array.get()->markers[i].pose.position.y < -0.05){
//	  object_list.remove(i);
//	}
//      }
//    }
//  }
  
//  if (object_list.size() == 1) {
//    return object_list.back();
//  }
//  else {
//    return 6;
//  }
//}

void publisher(const sensor_msgs::PointCloud2ConstPtr &input) {
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
  
  double x = 0.0;
  double y = 0.0;
  int numPts = 0;

  for (int i = 0; i < temp_cloud->points.size(); i++) {
    x += temp_cloud->points[i].x;
    y += temp_cloud->points[i].y;
    numPts++;
  }
  
  pcl::PointXYZ centroid;
  centroid.x = x / numPts;
  centroid.y = y / numPts;

  //std::cout << "numPts: " << numPts << " x: " << centroid.x << " y: " <<centroid.y << "\n";

  int maxTheta = 18000;
  int maxXRange = 3000;
  std::vector<std::vector<int>> accu(maxXRange,std::vector<int>(maxTheta));

  for (int j = 0; j < temp_cloud->points.size(); j++) {
    for (int theta = 0; theta < maxTheta; theta++) {
      double rho = temp_cloud->points[j].y * cos(theta * pi / 18000) + temp_cloud->points[j].x * sin(theta * pi / 18000);
      int rho_round = round(rho * 1000);
      if (rho_round > 0) {
        accu[rho_round][theta]++;
      }
    }
  }
  
  int max = 0;
  std::list<int> rList;
  std::list<int> tList;

  for (int k = 0; k < maxXRange; k++) {
    for (int l = 0; l < maxTheta; l++) {
      //std::cout << accu[k][l] << ",";
      if (accu[k][l] > max) {
        max = accu[k][l];
      }
    }
  }
  for (int k = 0; k < maxXRange; k++) {
    for (int l = 0; l < maxTheta; l++) {
      if (accu[k][l] == max) {
        rList.push_back(k);
	tList.push_back(l);
      }
    }
  }
  
  rList.sort();
  tList.sort();
  
  int rArr[rList.size()], tArr[tList.size()];
  std::copy(rList.begin(), rList.end(), rArr);
  std::copy(tList.begin(), tList.end(), tArr);
  int count = 0;
  double tempr, tempt, pmaxr, pmaxt = 0;
  
  for (int m = 0; m < rList.size(); m++) {
    tempr += rArr[m];
    tempt += tArr[m];
    count ++;
  }

  pmaxr = (tempr / count) / double(1000);
  pmaxt = (90 - (tempt / count) / double(100)) * pi / 180;

  geometry_msgs::Twist msg;
  msg.linear.x = centroid.x;
  msg.linear.y = centroid.y;
  msg.angular.x = pmaxt;
  
  pub_carInFront.publish(msg);
  //std::cout << "max: " << max << " rho:" << pmaxr << " theta:" << pmaxt << "\n";
  std::cout << "x: " << centroid.x << " y: " <<centroid.y << " theta: " << pmaxt << "\n";
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  
//  if (!mainMSG) {
//    mainMSG = ros::topic::waitForMessage<visualization_msgs::MarkerArray>("/viz");

//    objectNumberCar = initialiser(mainMSG);
//    objectTopicCar = "cluster_" + std::to_string(objectNumberCar);
//  }

  pub_carInFront = n.advertise<geometry_msgs::Twist>("dataInFrontOf1",1);
  ros::Subscriber sub = n.subscribe("PCLinFrontOf1", 1, publisher);
  ros::spin();

  return 0;
}

