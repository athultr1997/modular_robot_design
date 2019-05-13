#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_assembler/AssembleScans2.h>

// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl_ros/point_cloud.h>

using namespace laser_assembler;

int main(int argc, char *argv[])
{
	ros::init(argc,argv,"point_cloud_assembler");
	ros::NodeHandle nh;

	ros::service::waitForService("assemble_scans2");
	ros::ServiceClient client = nh.serviceClient<AssembleScans2>("assemble_scans2");
	AssembleScans2 srv;

	ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("kinect_robot_1_1/point_cloud_assembled",5,true);

	srv.request.begin = ros::Time(0,0);

	// if(client.call(srv)){
	// 	// printf("Got cloud with %u points\n", srv.response.cloud.points.size());
	// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_assembled(new pcl::PointCloud<pcl::PointXYZ>());
	// 	pcl::fromROSMsg(srv.response.cloud,*cloud_assembled);
	// 	ROS_INFO("Number of Cloud Points =%d \n",cloud_assembled->width);
	// }
	// else{
	// 	printf("Service call failed\n");
	// }
	ros::Rate loop_rate(1);
	while(ros::ok()){
		srv.request.end = ros::Time::now();
		if(client.call(srv)){
			pub.publish(srv.response.cloud);
			ROS_INFO("Number of Cloud Points =%d \n",srv.response.cloud.width);
		}
		else{
			ROS_INFO("Service call failed\n");
		}
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}