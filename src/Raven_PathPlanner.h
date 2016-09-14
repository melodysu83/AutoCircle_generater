#ifndef RAVEN_PATHPLANNER_H_
#define RAVEN_PATHPLANNER_H_

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

#define LEFT_ARM 0
#define RIGHT_ARM 1

#define MAX_RADIUS 10  // 10 different radius levels
#define MAX_SPEED 60   // 60 different speed levels
#define MIN_RADIUS 1
#define MIN_SPEED 1

#define RADIUS_level_TO_microm 3000  //in micro meter (= 3mm = 0.3cm)

#define DEL_POS_THRESHOLD 180	// in micro meter (= 0.18mm = 0.018cm)
#define STATE_THRESHOLD 300
#define DEL_ROT_THRESHOLD 2.5 	// in degrees
#define ROS_PUBLISH_RATE 1000 	// in Hz


using namespace raven_2;
using namespace std;


enum PATH_STATE{
	AROUND_CIRCLE,	// go along circle trajectory
	MOVETO_CIRCLE	// move to the new circle (when radius or center change)
};

class Raven_PathPlanner
{
	private:
		tf::Vector3 Center;		// the center of the circle
						// [NOTE]: good center (-85126,-22305,43358)
		tf::Vector3 Current_Pos;	// current raven position
		tf::Vector3 Delta_Pos;
		tf::Quaternion Current_Ori;	// current raven rotation

		tfScalar Radius;		// in mm
		tfScalar Speed;
		tfScalar Distance;		// the distance between current pos and center
		tfScalar Error;			// the difference between radius and distance
		PATH_STATE PathState;
	
		pthread_mutexattr_t data1MutexAttr;
		pthread_mutex_t data1Mutex;

		int Direction;
		int ArmType;
		bool FIRST_SEND;
		tfScalar last_y,last_z;
		tfScalar Kp;
		tfScalar sign(tfScalar);
		void checkPathState();
		void AutoCircleMotion1();		// algorithm 1 : kind of unstable
		void AutoCircleMotion2();		// algorithm 2 : better!
		void AutoCircleMotion3();		// algorithm 3 : even better! 
		tf::Vector3 AutoCircleMotion4();	// algorithm 4 : the best so far! (in use!!)	
		tf::Vector3 TuneRadiusMotion();

	public:
		Raven_PathPlanner();
		bool set_Radius(int);
		bool set_Speed(int);
		bool set_Direction(int);
		bool set_ArmType(int);
		bool set_Center(boost::array<int, 6>);
		bool set_Current_Pos(boost::array<int, 6>);
		bool set_Current_Ori(boost::array<float, 18>);
		tfScalar get_Radius();
		tfScalar get_Speed();
		void show_PathState();
		void show_Distance();
		void show_Center();
		void show_delPos();

		tfScalar DistanceOf(tf::Vector3,tf::Vector3);
		tf::Transform ComputeCircleTrajectory();
		tf::Transform ComputeNullTrajectory();
};
#endif
