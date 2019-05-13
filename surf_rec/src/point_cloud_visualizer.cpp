#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

void visualizer_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
	ROS_INFO("Working...\n");
	
}

int main(int argc,char *argv[])
{
	ros::init(argc,argv,"point_cloud_visualizer");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("point_cloud_input",10,visualizer_callback);

	ros::spin();
	
	return 0;
}