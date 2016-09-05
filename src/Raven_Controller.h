#ifndef RAVEN_CONTROLLER_H_
#define RAVEN_CONTROLLER_H_

#include <ros/ros.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <tf/transform_datatypes.h>
#include <sstream>
#include <pthread.h>
#include <stdio.h>
#include <iomanip>
#include <termios.h>
#include <queue>
#include "raven_2/raven_automove.h"
#include "raven_2/raven_state.h"


#define DEL_POS_THRESHOLD 3	// in mm
#define DEL_ROT_THRESHOLD 2.5 	// in degrees
#define ROS_PUBLISH_RATE 100 	// in Hz

#define MAX_RADIUS 10
#define MAX_SPEED 10
#define MIN_RADIUS 1
#define MIN_SPEED 1
#define LEFT_ARM 0
#define RIGHT_ARM 1


using namespace raven_2;
using namespace std;


class Raven_Controller
{
	private:
		int RADIUS;
		int SPEED;
		int DIRECTION;
		int PUB_COUNT;
		int SUB_COUNT;
		bool SHOW_STATUS;
		bool TERMINATED;
		bool PAUSE;
		bool SEND_NULL;

		pthread_t console_thread;
		pthread_t ros_thread;

		ros::Publisher RavenAutomove_publisher;
		ros::Subscriber RavenState_subscriber;

		raven_state CURR_RAVEN_STATE;
		tf::Transform TF_INCR[2];
		


	public:
		enum CTRL_MODE{
			SET_CENTER, 		// to define the center of circle
			CIRCLE_TRAJECTORY,	// to move along circle trajectory
			NULL_TRAJECTORY		// to stay stationary
		};

		Raven_Controller();		// constructor
		void initial(int, char**);	// initialization and console display
		void init_sys();
		bool init_ros(int, char**);
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
		void output_PUBinfo();
		void output_SUBinfo();

		int getKey();

}; //end of class definition

#endif
