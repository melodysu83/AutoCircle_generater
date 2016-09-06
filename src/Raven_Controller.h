#ifndef RAVEN_CONTROLLER_H_
#define RAVEN_CONTROLLER_H_

#include "Raven_PathPlanner.h"

class Raven_Controller
{
	private:
		int RADIUS;
		int SPEED;
		int DIRECTION;
		int PUB_COUNT;
		int SUB_COUNT;
		bool SHOW_STATUS;
		bool RECEIVED_FIRST;
		bool PAUSE;

		pthread_t console_thread;
		pthread_t ros_thread;

		ros::Publisher RavenAutomove_publisher;
		ros::Subscriber RavenState_subscriber;

		raven_state CURR_RAVEN_STATE;
		tf::Transform TF_INCR[2];
		
		Raven_PathPlanner LEFT_PATH;
		Raven_PathPlanner RIGHT_PATH;

	public:
		Raven_Controller();		// constructor
		void initial(int, char**);	// initialization and console display
		void init_sys();
		bool init_ros(int, char**);
		void init_pathplanner();
		void init_words();
		bool menu_words(bool);
		void final_words();

		void start_thread();		// thread management
		void join_thread();
		void *console_process(void);
		void *ros_process(void);
		static void *static_console_process(void*);
		static void *static_ros_process(void*);

		void publish_raven_automove();			 // ROS publish
		void callback_raven_state(raven_2::raven_state); // ROS subscribe

		void output_STATUS();		// show ROS and raven state
		void output_PATHinfo();
		void output_PUBinfo();
		void output_SUBinfo();

		int getKey();

}; //end of class definition

#endif
