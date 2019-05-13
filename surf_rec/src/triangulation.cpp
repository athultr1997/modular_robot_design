#include <iostream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/filters/voxel_grid.h>

#include <string>

std::string output_file_name;

pcl::PointCloud<pcl::PointXYZ>::Ptr voxelgrid_filter(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  ROS_INFO("Filteration started...");
  ROS_INFO("Point cloud size before filtering:%d",cloud->height*cloud->width);

  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  sor.setLeafSize(0.05f,0.05f,0.05f);
  sor.filter(*cloud);

  ROS_INFO("Point cloud size after filtering:%d",cloud->height*cloud->width);

  return cloud;
}

void greedy_surf_rec(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // Normal estimation
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  n.setKSearch (20);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
  // cloud_with_normals = cloud + normals

  // Create search tree
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.5);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // // Additional vertex information
  // std::vector<int> parts = gp3.getPartIDs();
  // std::vector<int> states = gp3.getPointStates();

  cout<<"Mesh details:"<<triangles.header<<"\n";
  // Saving file
  pcl::io::savePolygonFileSTL(output_file_name,triangles,false);
  pcl::io::savePolygonFileSTL(output_file_name.substr(0, output_file_name.size()-4)+"_binary.stl",triangles,true);  

  // Console Out
  // std::vector<int>::iterator it;

  // for(it=parts.begin();it<parts.end();it++)
  // {
  //   std::cout<<*it<<" ";
  // }

  // std::cout<<"\n\n";

  // for(it=states.begin();it<states.end();it++)
  // {
  //   std::cout<<*it<<" ";
  // }

  // std::cout<<"\n";

}


void traingulation_callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  ROS_INFO("Calllback started...");
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*cloud_msg,*cloud);

  // Load input file into a PointCloud<T> with an appropriate type
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PCLPointCloud2 cloud_blob;
  // pcl::io::loadPCDFile ("/home/athultr/catkin_ws/src/surf_rec/examples/inputs/bunny.pcd", cloud_blob);
  // pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
  
  //Filtering using VoxelGrid Filter
  cloud = voxelgrid_filter(cloud);

  //Surface Reconstruction using greedy projection
  greedy_surf_rec(cloud);

  ros::shutdown();  
}

int main (int argc, char** argv)
{
  ros::init(argc,argv,"triangulation");
  ros::NodeHandle nh;

  if(argc==2){
    output_file_name = argv[1];
    output_file_name = "/home/athul/catkin_ws/src/surf_rec/examples/outputs/" + output_file_name;
  }
  else{
    output_file_name = "/home/athul/catkin_ws/src/surf_rec/examples/outputs/kinect_output.stl";
  }

  ros::Subscriber sub = nh.subscribe("point_cloud_input",10,traingulation_callback);

  ros::spin();
  
  return 0;
}