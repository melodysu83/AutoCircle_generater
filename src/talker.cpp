#include "Raven_Controller.h"

int main(int argc, char **argv)
{
	
	Raven_Controller ctrl;

	// initialize the system
	ctrl.initial(argc,argv); 
  	
	// start the console_thread and ros_thread
	ctrl.start_thread(); 
	
	// trigger ROS publish and subscribe update
	ros::spin();

	// join the console_thread and ros_thread
	ctrl.join_thread();	

  	exit(1);
}
