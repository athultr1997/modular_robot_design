#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

std::string save_file_name;

void save_file_pcd(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) 
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::fromROSMsg(*cloud_msg,*cloud);
	ROS_INFO("Header:%s\n",cloud_msg->header.frame_id.c_str());
	pcl::io::savePCDFileASCII (save_file_name, *cloud);
	ROS_INFO("Point Cloud Saved as PCD file...\n");
	ros::shutdown();
}

int main(int argc, char *argv[])
{ 
	ros::init(argc,argv,"save_pclmsg_pcdfile");

	if(argc==2){
		save_file_name = argv[1];
		save_file_name = "/home/athul/catkin_ws/src/surf_rec/examples/inputs/" + save_file_name;
	}
	else{
		save_file_name = "/home/athul/catkin_ws/src/surf_rec/examples/inputs/kinect_data_assembled.pcd";
	}

	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("kinect_robot_1_1/point_cloud_assembled",10,save_file_pcd);

	ros::spin();

	return 0;
}