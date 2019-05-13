#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/tf.h>

void call_back(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	ROS_INFO_STREAM( feedback->marker_name << " is now at "
		<< feedback->pose.position.x << ", " << feedback->pose.position.y
		<< ", " << feedback->pose.position.z );
}

int main(int argc, char *argv[])
{
	ros::Time::init();
	ros::init(argc,argv,"rviz_user_input");

	//menu which appears when right clicking the marker
	interactive_markers::MenuHandler menu_handler;

    //normal marker of red colour 
	visualization_msgs::Marker sphere_marker;
	sphere_marker.type = visualization_msgs::Marker::SPHERE;
	sphere_marker.scale.x = 0.25;
	sphere_marker.scale.y = 0.25;
	sphere_marker.scale.z = 0.25;
	sphere_marker.color.r = 1.0;
	sphere_marker.color.g = 0.0;
	sphere_marker.color.b = 0.0;
	sphere_marker.color.a = 1.0;

	//non-interactive control for containing the marker
	visualization_msgs::InteractiveMarkerControl sphere_container_control;
	sphere_container_control.always_visible = true;
	sphere_container_control.markers.push_back(sphere_marker);

    //control for the marker in x-direction
	visualization_msgs::InteractiveMarkerControl x_axis_control;
	x_axis_control.name = "move_3D_control";
	x_axis_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
	x_axis_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	x_axis_control.always_visible = true;

	//control for the marker in y-direction
	visualization_msgs::InteractiveMarkerControl y_axis_control;
	y_axis_control.name = "move_3D_control";
	y_axis_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
	y_axis_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	y_axis_control.always_visible = true;
	y_axis_control.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,1.57);

	//control for the marker in z-direction
	visualization_msgs::InteractiveMarkerControl z_axis_control;
	z_axis_control.name = "move_3D_control";
	z_axis_control.orientation_mode = visualization_msgs::InteractiveMarkerControl::INHERIT;
	z_axis_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
	z_axis_control.always_visible = true;
	z_axis_control.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,-1.57,0);

	//interactive marker = marker + non_interactive_control + control1 + control2 + control3
	visualization_msgs::InteractiveMarker TSL;
	TSL.header.frame_id = "/map";
	TSL.header.stamp = ros::Time::now();
	TSL.name = "TSL";
	TSL.description = "TSL";
	TSL.scale = 0.25;
	TSL.controls.push_back(sphere_container_control);
	TSL.controls.push_back(x_axis_control);
	TSL.controls.push_back(y_axis_control);
	TSL.controls.push_back(z_axis_control);

	//server for handling feedbacks from Rviz
	interactive_markers::InteractiveMarkerServer server("user_input_handler");
	server.insert(TSL, &call_back);
	server.applyChanges();

	ros::spin();

	return 0;
}
