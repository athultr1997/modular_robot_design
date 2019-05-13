#include "URDF.h"
#include "iostream"

URDF::URDF()
{
	package_name = "surf_rec";

}

void URDF::input(std::string input_file_name)
{
	if(!input_file_name.empty())
	{
		std::string path = ros::package::getPath(package_name);
		input_file_name = path + "/examples/DH_parameters/" + input_file_name;
	}
	else
	{
		std::cout<<"Please enter the input file name.\n Usage: rosrun surf_rec urdf_generator <input_file_name>"
	}

	fin.open(input_file_name,std::ios::in);

}

void URDF::build_urdf_file()
{
	
}