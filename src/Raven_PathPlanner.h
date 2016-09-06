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
#include "raven_2/raven_automove.h"
#include "raven_2/raven_state.h"


#define LEFT_ARM 0
#define RIGHT_ARM 1

#define MAX_RADIUS 10  // 10 different radius levels
#define MAX_SPEED 10   // 10 different speed levels
#define MIN_RADIUS 1
#define MIN_SPEED 1

#define RADIUS_level_TO_microm 10000  //in micro meter (= 10mm = 1cm)

#define DEL_POS_THRESHOLD 3000	// in micro meter (= 3mm = 0.3cm)
#define DEL_ROT_THRESHOLD 2.5 	// in degrees
#define ROS_PUBLISH_RATE 100 	// in Hz

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
		tf::Vector3 Current_Pos;	// current raven position
		tf::Vector3 Delta_Pos;
		tf::Quaternion Current_Ori;	// current raven rotation

		tfScalar Radius;		// in mm
		tfScalar Speed;
		tfScalar Distance;		// the distance between current pos and center
		PATH_STATE PathState;
	
		int Direction;
		int ArmType;
		void checkPathState();
		void AutoCircleMotion1();	// algorithm 1 
		void AutoCircleMotion2();	// algorithm 2 : better!

	public:
		Raven_PathPlanner();
		bool set_Radius(int);
		bool set_Speed(int);
		bool set_Direction(int);
		bool set_ArmType(int);
		bool set_Center(boost::array<int, 6>);
		bool set_Current_Pos(boost::array<int, 6>);
		bool set_Current_Ori(boost::array<float, 18>);
		void show_PathState();
		void show_Center();
		void show_delPos();

		tfScalar DistanceOf(tf::Vector3,tf::Vector3);
		tf::Transform ComputeCircleTrajectory();
		tf::Transform ComputeNullTrajectory();
};
#endif
