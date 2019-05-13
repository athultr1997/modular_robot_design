#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>

#include <fstream>
#include "iostream"
#include <queue>
#include <vector>
#include <stack>

// #include "URDF.h"

/*
The Node objects are used to make a tree representing the DOM
Depth search of the resulting tree will make the XML file
The opening tag is written on the first visit to the node
The closing tag is written on the second/last visit to the node
Leaf nodes are closed in the same line
*/
class Node
{
	/*
	Node class represents an element of DOM
	*/
	public:
		std::string name;
		std::vector<std::string> attributes;//attributes of the element
		std::vector<std::string> attribute_values;//value of the attribute
		std::vector<Node *> children;//sub-elements of the element represented by node
		Node *parent;//parent element of the element represented by node

		Node();
		~Node();
};

Node::Node()
{
	parent = NULL;
}

Node::~Node()
{
   for(std::vector<Node *>::iterator p = children.begin();p != children.end(); ++p)
   {
      delete *p; 
   }
   children.clear();            
}

class URDF
{
	private:
		std::ifstream fin;
		std::ofstream fout;
		std::string package_name;
		int DoF;
		std::string input_file_name;
		tf::Point base_location;
		double **DH_parameters;
		Node *root;
		Node *reference_pointer;

	public:
		URDF();
		~URDF();
		bool input(std::string input_file_name);
		void make_urdf_file();
		void display_DH_parameters();
		void display_URDF_tree(Node *p,int intend);
		
		void initialize_document(std::string robot_name);
		void add_node(std::string name,std::string data,Node *parent);
		void depth_traversal(Node *ptr,int index);

		//Function for writing the tags of the element to the XML file
		//If status is true, the opening tag is written
		//If status is false, the closing tag is written		
		void write_node(Node *ptr,bool status,int index);
};

URDF::URDF()
{
	package_name = "surf_rec";
	DoF = 0;
	root = NULL;
}

URDF::~URDF()
{
	//closing the input file
	fin.close();
	//closing the output (urdf) file
	fout.close();

	//deallocating memory of the 2D array for storing DH_parameters
	for(int i=0;i<DoF;i++)
	{
		delete [] DH_parameters[i];
		DH_parameters[i] = NULL;
	}
}

void URDF::initialize_document(std::string robot_name="robot")
{
	//Adding preamble to the XML file
	fout<<"<?xml version=\"1.0\" ?>\n";

	root = new Node;
	reference_pointer = root;

	//Adding the robot element which is the root of the tree
	//Keeping the name of the robot as the name of the input_file_name as default
	reference_pointer->name = "robot";
	reference_pointer->attributes.push_back("name");
	reference_pointer->attribute_values.push_back(robot_name); 
}

void URDF::add_node(std::string name = "link",std::string data = "",Node *parent = NULL)
{
	Node *p = new Node;
	p->name = name;
	if(parent==NULL)
	{
		parent = reference_pointer;
		p->parent = parent;
	}
	else
	{
		p->parent = parent;
	}
	parent->children.push_back(p);

	if(!data.empty())
	{
		std::string delimiter_1 = "=";
		std::string delimiter_2 = ";";
		int pos = 0;

		while(!data.empty())
		{
			pos = data.find(delimiter_1);
			p->attributes.push_back(data.substr(0,pos));
			data = data.erase(0,pos+delimiter_1.length());
			pos = data.find(delimiter_2);
			p->attribute_values.push_back(data.substr(0,pos));
			data = data.erase(0,pos+delimiter_2.length());
		}
	}

	reference_pointer = p;
}

void URDF::display_DH_parameters()
{
	if(DH_parameters==NULL)
	{
		std::cout<<"Input file have not been specified yet."
		<<"Please use the input member function for initialization\n";
	}
	else
	{
		std::cout<<"\n";
		for(int i=0;i<DoF;i++)
		{
			for(int j=0;j<3;j++)
			{
				std::cout<<DH_parameters[i][j]<<"|";
			}
			std::cout<<"\n";
		}
	}
}

void URDF::display_URDF_tree(Node *p=NULL,int intend = 0)
{
	p = root;	
    if(p==NULL)
    {
    	std::cout<<"The URDF tree is empty\n";
    	return;
    }

    std::cout<<"\n";
    // Standard level order traversal code using queue
    std::queue <Node *> q;//Creating a Queue
    q.push(p);//Inserting the root node
    while(!q.empty())// loop as long as the queue is not empty
    {
    	int n = q.size();

    	//If the node has children
    	while(n>0)
    	{
    		// Dequeue an item from queue and print it 
    		Node *tmp_ptr = q.front();
    		q.pop();
    		std::cout<<tmp_ptr->name<<" ";

    		// Enqueue all children of the dequeued node
    		for(int i=0;i<tmp_ptr->children.size();i++)
    		{
    			q.push(tmp_ptr->children[i]);
    		}
    		n--;
    	}
    	std::cout<<"\n";//print a new line between two levels
    }
}

bool URDF::input(std::string file_name)
{	
	//temperory variables for data handlig
	std::string tmp;
	int pos;
	std::string delimiter = "|";

	if(!file_name.empty())
	{
		std::string path = ros::package::getPath(package_name);
		input_file_name = path + "/examples/DH_parameters/" + file_name;
		fin.open(input_file_name,std::ios::in);
	}
	else
	{
		std::cout<<"Please enter the input file name.\nUsage: rosrun surf_rec urdf_generator <input_file_name>\n";
		return false;
	}

	pos = input_file_name.find(".");
	fout.open(input_file_name.substr(0,pos) + ".urdf");

	if(fin)
	{
		//taking in the degree of freedom
		getline(fin,tmp);
		DoF = std::stoi(tmp);
		DH_parameters = new double* [DoF];
		for(int i=0;i<DoF;i++)
		{
			DH_parameters[i] = new double[3];
		}
	}
	
	if(fin)
	{
		//taking in the base location
		getline(fin,tmp);
		pos = tmp.find(delimiter);
		base_location.setX(std::stod(tmp.substr(0,pos)));
		tmp = tmp.erase(0,pos+delimiter.length());
		pos = tmp.find(delimiter);
		base_location.setY(std::stod(tmp.substr(0,pos)));
		tmp = tmp.erase(0,pos+delimiter.length());
		base_location.setZ(std::stod(tmp));
	}
	
	if(fin)
	{
		//taking in the empty line
		getline(fin,tmp);
	}

	int i = 0;
	getline(fin,tmp);
	
	while(fin||i<DoF)
	{
		for(int j=0;j<3;j++)
		{
			pos = tmp.find(delimiter);
			DH_parameters[i][j] = std::stod(tmp.substr(0,pos));
			tmp = tmp.erase(0,pos+delimiter.length());
		}
		getline(fin,tmp);
		i++;
	}

	return true;
}

void URDF::write_node(Node *ptr,bool status,int index)
{
	std::string space = "";
	for(int i=0;i<index;i++)
	{
		space += "  "; 
	}

	if(status)
	{
		std::cout<<space<<"<"<<ptr->name;
		fout<<space<<"<"<<ptr->name;
		for(int i=0;i<ptr->attributes.size();i++)
		{
			std::cout<<" "<<ptr->attributes[i]<<"="<<"\""<<ptr->attribute_values[i]<<"\"";
			fout<<" "<<ptr->attributes[i]<<"="<<"\""<<ptr->attribute_values[i]<<"\"";
		}
		if(ptr->children.size()!=0)
		{
			std::cout<<">\n";
			fout<<">\n";
		}
		else
		{
			std::cout<<"/>\n";	
			fout<<"/>\n";
		}
	}
	else
	{
		std::cout<<space<<"</"<<ptr->name<<">\n";
		fout<<space<<"</"<<ptr->name<<">\n";
	}
}

void URDF::depth_traversal(Node *ptr,int index=0)
{
	//using post-order scheme
	write_node(ptr,true,index);
	for(std::vector<Node *>::iterator p = ptr->children.begin();p != ptr->children.end(); ++p)
	{
		depth_traversal(*p,index+1);
	}
	if(ptr->children.size()!=0)
	{
		write_node(ptr,false,index);
	}
}

void URDF::make_urdf_file()
{
	if(DH_parameters==NULL)
	{
		std::cout<<"Input file have not been given yet."
		<<"Please use the input member function for initialization\n";
	}
	else
	{	
		int pos = input_file_name.find_last_of("/");
		std::cout<<input_file_name.substr(pos+1,input_file_name.find("."));
		initialize_document(input_file_name.substr(pos+1,input_file_name.find(".")-pos-1));

		// //psuedo-link for fixing the model to the world for Gazebo
		// add_node("link","name=world;",reference_pointer);
		// reference_pointer = root;
		//for storing the link length of each link
		double length;
		tf::Pose Transformation;

		//loop for adding links
		for(int i=0;i<DoF;i++)
		{
			reference_pointer = root;

			if(i==DoF-1)
			{
				//the length of the last link is not specified in DH-parameters hence it is taken as 1
				length = 1.0;
			}
			else
			{
				//length of the link is determined by the parameter 'a' in DH-parameters
				length = DH_parameters[i+1][1];	
			}
			

			//commands for adding a link
			//reference_pointer gets set to the newly created node, hence
			//it is necessary to reset to the parent node to add sibling nodes
			add_node("link","name=link_"+std::to_string(i+1)+";",reference_pointer);
			
			// //commands for adding the inertial tag and its elements
			// add_node("inertial","",reference_pointer);
			// add_node("mass","value=1;",reference_pointer);
			// add_node("origin","rpy=0  0  0;xyz=0  0  0;",reference_pointer->parent);
			// add_node("inertia","ixx=0.14;ixy=0;ixz=0;iyy=0.14;iyz=0;izz=0.14;",reference_pointer->parent);

			// //commands for adding the collision tag and its elements
			// add_node("collision","name=collision"+std::to_string(i+1)+";",reference_pointer->parent->parent);
			// add_node("origin","rpy=0  0  0;xyz=0  0  0;",reference_pointer);
			// add_node("geometry","",reference_pointer->parent);
			// add_node("cylinder","length="+std::to_string(length)+";radius=0.1;",reference_pointer);

			// commands for adding the visual tag and its elements
			// add_node("visual","name=visual"+std::to_string(i+1)+";",reference_pointer->parent->parent->parent);
			// add_node("origin","rpy=0  0  0;xyz=0  0  0;",reference_pointer);
			// add_node("geometry","",reference_pointer->parent);
			// add_node("cylinder","length="+std::to_string(length)+";radius=0.1;",reference_pointer);
		}

		// //joint for rigidly fixing the robot to ground in Gazebo
		// reference_pointer = root;
		// add_node("joint","name=joint_"+std::to_string(0)+";type=fixed;",reference_pointer);
		// add_node("parent","link=world;",reference_pointer);
		// add_node("child","link=link_1;",reference_pointer->parent);

		//loop for adding joints
		for(int i=0;i<DoF-1;i++)
		{
			reference_pointer = root;
			//defining the tranformation between the two links being joined
			tf::Pose rot_alpha(tf::Quaternion(tf::Vector3(1,0,0),DH_parameters[i+1][0]),tf::Vector3(0,0,0));
			tf::Pose trans_a(tf::Quaternion(0,0,0,1),tf::Vector3(DH_parameters[i+1][1],0,0));
			tf::Pose rot_theta(tf::Quaternion(tf::Vector3(0,0,1),0),tf::Vector3(0,0,0));
			tf::Pose trans_d(tf::Quaternion(0,0,0,1),tf::Vector3(0,0,DH_parameters[i+1][2]));
			
			Transformation = rot_alpha * trans_a * rot_theta * trans_d;
			tf::Point origin_coordinates = Transformation * tf::Vector3(0,0,0); 

			// commands for adding joints
			add_node("joint","name=joint_"+std::to_string(i+1)+";type=continuous;",reference_pointer);
			add_node("parent","link=link_"+std::to_string(i+1)+";",reference_pointer);
			add_node("child","link=link_"+std::to_string(i+2)+";",reference_pointer->parent);
			add_node("origin","rpy=0  0  0;xyz="+std::to_string(origin_coordinates.getX())+"  "
				+std::to_string(origin_coordinates.getY())+"  "+std::to_string(origin_coordinates.getZ())
				+";",reference_pointer->parent);
			add_node("axis","xyz=0  0  1;",reference_pointer->parent);
		}

		depth_traversal(root);
	}
}

int main(int argc, char *argv[])
{
	ros::init(argc,argv,"urdf_generator");
	ros::NodeHandle nh;

	URDF robot;

	bool result;

	if(argc==2){
		result = robot.input(argv[1]);				
	}
	else{
		result = robot.input("");
	}

	if(!result)
	{
		ros::shutdown();
	}

	robot.display_DH_parameters();
	robot.make_urdf_file();
	// robot.display_URDF_tree();
		
	return 0;
}