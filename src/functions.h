#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

#include <string>
#include <vector>
#include <map>
#include <ostream>
#include <tf/transform_datatypes.h>
#include <stdio.h>
#include <iomanip>
#include <ros/ros.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <sstream>
#include <pthread.h>
#include <termios.h>
#include <queue>
#include "/home/biorobotics/rosworkspace/src/raven_2/msg_gen/cpp/include/raven_2/raven_automove.h"
#include "/home/biorobotics/rosworkspace/src/raven_2/msg_gen/cpp/include/raven_2/raven_state.h"


#define maxin 10


using namespace raven_2;
using namespace std;

class functions{
	public: 
	unsigned long int gtime;
	functions();
	ros::Publisher pub_automove;
	tf::Vector3 last_pos;
	tf::Vector3 current_pos;
	tf::Vector3 des_pos;
	tf::Vector3 increment;
	tf::Transform TF_INCR[2];
	void init_ros(int, char **);
	void publish_automove(tf::Vector3 , int) ;
	void Callback(raven_2::raven_state);

};
#endif
