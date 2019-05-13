#ifndef URDF_H
#define URDF_H 

#include <fstream>

class URDF
{
	private:
		ifstream fin;
		std::string package_name;		
	public:
		URDF();
		void input(std::string input_file_name);
		void build_urdf_file();

};

#endif