#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"rec_surf_generator");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("visualization_marker",1,true);

    std::string mesh_file = "package://surf_rec/examples/meshes_generated/20_2_mesh_binary.stl";

    if(argc==2)
    {
        mesh_file = "package://surf_rec/examples/meshes_generated/" + (std::string)argv[1];
    }

    //normal marker for visualizing the stl workspace file
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "workspace";
    marker.id = 0;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0);
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = mesh_file;
    marker.mesh_use_embedded_materials = true;

    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;
    
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    //White Colour
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    pub.publish(marker);

    ros::spin();

    return 0;
}