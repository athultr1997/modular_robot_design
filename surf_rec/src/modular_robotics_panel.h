#ifndef MODULAR_ROBOTICS_PANEL_H
#define MODULAR_ROBOTICS_PANEL_H

#ifndef Q_MOC_RUN
# include <ros/ros.h>
# include <rviz/panel.h>
#endif

namespace modular_robotics
{
	class ModularRoboticsPanel: public rviz::Panel
	{
		// This class will use slots and signals hence 
		// meta-object features should be enabled using 
		// the macro below in the private section
		Q_OBJECT

		public:
			// The parent class rviz::Panel is a subclass of QWidget
			// and hence to use the same constructor in ModularRoboticsPanel
			ModularRoboticsPanel(QWidget *parent=0);

			// Overriding the load and save functions of rviz::Panel
			virtual void load( const rviz::Config& config );
  			virtual void save( rviz::Config config ) const;

  		Q_SIGNALS:
  			void send_TSL_from_text_input(double x,double y,double z);
  			void send_TSL_from_gui_input(double x,double y,double z);

		public Q_SLOTS:
			void set_TSL();

		protected:
			// '_' at the end denotes data members of the class

			// ROS node handle for subscribing to topics,...
			ros::NodeHandle nh_;

			// Publisher for sending TSL position to interactive marker
			ros::Publisher poistion_publisher_;

			// (x,y,z) coordinates denoting the position of the interactive marker
			double x_, y_, z_;
			
	}

} // End of modular_robotics_plugin namespace 

#endif