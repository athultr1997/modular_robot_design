#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

ros::Publisher pub;

void frame_change_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*cloud_msg,*cloud);

	cloud->header.frame_id = "kinect_link";
	pcl_conversions::toPCL(cloud_msg->header.stamp, cloud->header.stamp);

	pub.publish(cloud);
}

int main(int argc, char *argv[])
{	
	ros::init(argc,argv,"point_cloud_frame_change");
	ros::NodeHandle nh;

	pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("point_cloud_kinect_link_frame",10);
	ros::Subscriber sub = nh.subscribe("camera/depth/points",10,frame_change_callback);
	ros::spin();

	return 0;
}