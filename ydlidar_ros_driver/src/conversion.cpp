
#include <ros/ros.h>
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"

class PubAndSub
{
	private:
		ros::Subscriber pc_sub;
		ros::Publisher pc_pub;
		ros::NodeHandle nh;
		
	public:
		PubAndSub() 
		{
			pc_sub = nh.subscribe("point_cloud",1, &PubAndSub::callback, this);
			pc_pub = nh.advertise<sensor_msgs::PointCloud2>("point_cloud2",1);
		}

		void callback(const sensor_msgs::PointCloud& cloud_old)
		{
			sensor_msgs::PointCloud2 cloud_new;
		
			sensor_msgs::convertPointCloudToPointCloud2( cloud_old, cloud_new);
			pc_pub.publish(cloud_new);
		}
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "pointcloud_conversion");
  PubAndSub conversion;
                          
  ros::spin();

  return 0;
}


