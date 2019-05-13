#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

int main(int argc,char *argv[])
{
	ros::init(argc,argv,"point_cloud_generator");
	ros::NodeHandle nh;

	std::string input_file_name;

	if(argc==2){
		input_file_name = argv[1];
		input_file_name = "/home/athul/catkin_ws/src/surf_rec/examples/inputs/" + input_file_name;
	}
	else{
		input_file_name = "/home/athul/catkin_ws/src/surf_rec/examples/inputs/kinect_data_assembled.pcd";
	}


	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	if(pcl::io::loadPCDFile(input_file_name,*source_cloud)<0)
	{
		ROS_INFO("Error loading point cloud\n");
      	ros::shutdown();
	}
	
	ROS_INFO("point cloud loaded successfully!\n");
	ROS_INFO("Number of Cloud Points =%d \n",source_cloud->width);

	pcl_conversions::toPCL(ros::Time::now(), source_cloud->header.stamp);
	source_cloud->header.frame_id = "/map";
	ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("point_cloud_input",10,true);

	pub.publish(source_cloud);
	ROS_INFO("data published...\n");

	ros::spin();

	return 0;
}