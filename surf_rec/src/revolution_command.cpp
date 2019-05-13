#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char *argv[])
{
	ros::init(argc,argv,"revolution_command");
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<std_msgs::Float64>("kinect_robot_1_1/joint_0_position_controller/command",10,true);
	std_msgs::Float64 position_msg;
	float resolution = -0.174533;
	float position = 0;
	ros::Rate loop_rate(2);

	while(ros::ok() && position>-2.35619){
		position_msg.data = position;
		pub.publish(position_msg);
		ros::spinOnce();
		loop_rate.sleep();

		position += resolution;
	}

	return 0;
}