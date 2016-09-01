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
//#include "bullet/bullet3-2.83.7/src/LinearMath/btVector3.h"
//#include "bullet/bullet3-2.83.7/src/LinearMath/btTransform.h"
//#include "bullet/bullet3-2.83.7/src/LinearMath/btQuaternion.h"

#define ROS_PUBLISH_RATE 10
#define MAX_RADIUS 10
#define MAX_SPEED 10
#define MIN_RADIUS 1
#define MIN_SPEED 1

using namespace raven_2;
using namespace std;

//----------------------------global variables---------------------------
ros::Publisher AutoCircle_generator;
ros:: Subscriber RavenState_subscriber;

raven_2::raven_state CURR_RAVEN_STATE;

pthread_t console_thread;
pthread_t ros_thread;

tfScalar RADIUS, SPEED;
int DIRECTION;
bool SHOW_STATUS, SHOW_SUB, TERMINATED, PAUSE;

int32_t DEL_POS[6][360];
tf::Transform TF_INCR[2][360];

//--------------------------function declaration-------------------------

void computeNewTrajectory();
int init_sys();
bool init_ros();
bool init_process(int, char**);
void publish_raven_automove_ros(int , int );
void autoRavenStateCallback(raven_2::raven_state);
void displayRcvdMsg(int);
int getKey();
void outputSTATUS();
void *console_process(void*);
void *ros_process(void*);
void finalWords();

//--------------------------function definition--------------------------

/**
*	\fn void ComputeNewTrajectory()
*
* 	\brief generates pre-planned circle trajectory according to circle radius
*
* 	\param void
*
*	\return void
*/
void ComputeNewTrajectory()
{
	for(int j=0; j<360; j++)
	{
		for(int i=0; i<6; i++)
		{
			//DEL_POS (this seems to be actually unused)
			DEL_POS[i][j] = 0; //.. ?is that it
		}

		for(int i=0; i<2; i++)
		{
			//TF_INCR
			//..
			// tf::Vector3 tmpVec = (tf::Scalar& x, tf::Scalar& y, tf::Scalar& z);
			// tf::Quaternion q_temp = ;
			tfScalar X = RADIUS;
 			tfScalar Y = RADIUS;
			tfScalar Z = RADIUS;
			tf::Vector3 tmpVec(X,Y,Z); 
			
			tfScalar W = 1;
			tfScalar QX = 2;
			tfScalar QY = 3;
			tfScalar QZ = 4;			
			tf::Quaternion q_temp(QX,QY,QZ,W);

			TF_INCR[i][j].setOrigin(tmpVec); //add position increment
			TF_INCR[i][j].setRotation(q_temp); //add rotation increment
		}
	}	
}


/**
*	\fn int init_sys() 
*
*	\brief initialize ROS and connect the pub & sub relation.
*
* 	\param void
*
*	\return int
*/
int init_sys()  
{
	RADIUS = 1;
	SPEED = 1;
	DIRECTION = 1;
	SHOW_STATUS = false;
	SHOW_SUB = false;
	TERMINATED = false;
	PAUSE = false;
	ComputeNewTrajectory();

	string start = "0";
	do
	{
		cout<<endl<<endl;
		cout<<"Welcome to the Auto Circle Generator for RAVEN2"<<endl<<endl;
		cout<<"Default settings: RADIUS = "<<RADIUS<<endl;
		cout<<"                  SPEED = "<<SPEED<<endl;
		cout<<"                  DIRECTION = "<<DIRECTION<<endl<<endl;
		cout<<"Please press \"Enter\" to start!";
		getline(std::cin,start);
	}while(start!="");

	cout<<"Auto Circle Generator starting..."<<endl;
}


/**
*	\fn int init_ros(int argc, char** argv) 
*
*	\brief initialize ROS and connect the pub & sub relation.
*
* 	\param int argc, char** argv
*
*	\return int
*/
bool init_ros() 
{
	static ros::NodeHandle n;
  	AutoCircle_generator = n.advertise<raven_automove>("raven_automove", 1);
	RavenState_subscriber = n.subscribe<raven_state>("raven_state",1,autoRavenStateCallback,ros::TransportHints().unreliable());
	return true;
}

/**
*	\fn int init_process(int argc, char** argv) 
*
*	\brief initialize everything for this program.
*
* 	\param int argc, char** argv
*
*	\return int
*/
bool init_process(int argc, char **argv)
{
	ros::init(argc, argv, "AutoCircle_generator");
	init_sys();
	if(!init_ros())
  	{
     		cerr << "ERROR! Fail to init ROS. Exiting!\n";
		return false;
  	}
	return true;
}


/**
*	\fn void publish_raven_automove_ros()
*
*	\brief publish the auto circle command to raven_automove topic on ROS.
*
* 	\param void
*
*	\return void
*/
void publish_raven_automove_ros(int PUB_INDEX, int PUB_COUNT)
{
	static ros::Rate loop_rate(ROS_PUBLISH_RATE);
	static raven_automove msg_raven_automove;

	//hdr
	msg_raven_automove.hdr.stamp = msg_raven_automove.hdr.stamp.now();

	//del_pos
	for (int i=0;i<6;i++)
	{
		msg_raven_automove.del_pos[i] = DEL_POS[i][PUB_INDEX];
	}

	//tf_incr
	tf::transformTFToMsg(TF_INCR[0][PUB_INDEX], msg_raven_automove.tf_incr[0]);
	tf::transformTFToMsg(TF_INCR[1][PUB_INDEX], msg_raven_automove.tf_incr[1]);

	AutoCircle_generator.publish(msg_raven_automove);
	if(SHOW_STATUS)
	{
		ROS_INFO("talkerAutoCircle publish: raven_automove[%d]", PUB_COUNT);
		SHOW_SUB = true;
	}

	ros::spinOnce();

	for(int i = MAX_SPEED; i>=SPEED; i--)
		loop_rate.sleep();
}


/**
*	\fn void autoRavenStateCallback(raven_2::raven_state msg)
*
*	\brief This function is automatically called whenever someone publish to raven_state topic
*
* 	\param raven_2::raven_state msg
*
*	\return void
*/
void autoRavenStateCallback(raven_2::raven_state msg) 
{
	static int SUB_COUNT = 0;

	//save everything in the updated raven_state 
	//.. (some maybe uneccecary: ignore later...)

	CURR_RAVEN_STATE.runlevel = msg.runlevel;
	CURR_RAVEN_STATE.sublevel = msg.sublevel;
	CURR_RAVEN_STATE.last_seq = msg.last_seq;
	CURR_RAVEN_STATE.dt = msg.dt;

	for(int i=0; i<2; i++)
	{
		CURR_RAVEN_STATE.type[i] = msg.type[i];
		CURR_RAVEN_STATE.grasp_d[i] = msg.grasp_d[i];
	}

	for(int i=0; i<6; i++)
	{
		CURR_RAVEN_STATE.pos[i] = msg.pos[i];
		CURR_RAVEN_STATE.pos_d[i] = msg.pos_d[i];
	}

	for(int i=0; i<16; i++)
	{
		CURR_RAVEN_STATE.mpos[i] = msg.mpos[i];
		CURR_RAVEN_STATE.mpos_d[i] = msg.mpos_d[i];
		CURR_RAVEN_STATE.jpos[i] = msg.jpos[i];
		CURR_RAVEN_STATE.jpos_d[i] = msg.jpos_d[i];

		CURR_RAVEN_STATE.encVals[i] = msg.encVals[i];
		CURR_RAVEN_STATE.tau[i] = msg.tau[i];
		CURR_RAVEN_STATE.mvel[i] = msg.mvel[i];
		CURR_RAVEN_STATE.jvel[i] = msg.jvel[i];
		CURR_RAVEN_STATE.encoffsets[i] = msg.encoffsets[i];
	}
	
	for(int i=0; i<18; i++)
	{
		CURR_RAVEN_STATE.ori[i] = msg.ori[i];
		CURR_RAVEN_STATE.ori_d[i] = msg.ori_d[i];
	}

	//display the received raven_state data
	if(SHOW_STATUS && SHOW_SUB)
	displayRcvdMsg(SUB_COUNT);

	//update recieved data count
	SUB_COUNT ++;
}


/**
*	\fn void displayRcvdMsg(int SUB_COUNT)
*
*	\brief Display the recieved data from the Auto Circle Generator.
*
* 	\param int SUB_COUNT
*
*	\return void
*/
void displayRcvdMsg(int SUB_COUNT)
{
	//display the updated raven_state
	//.. modify(add more useful stuff)
	
	ROS_INFO("talkerAutoCircle subscribe: raven_state[%d]", SUB_COUNT);

	for(int i=0;i<3;i++)
	cout<<"\t"<<"pos["<<i<<"] = "<<CURR_RAVEN_STATE.pos[i];
	cout<<endl;
	for(int i=3;i<6;i++)
	cout<<"\t"<<"pos["<<i<<"] = "<<CURR_RAVEN_STATE.pos[i];
	cout<<endl;

	for(int i=0;i<3;i++)
	cout<<"\t"<<"pos_d["<<i<<"] = "<<CURR_RAVEN_STATE.pos_d[i];
	cout<<endl;
	for(int i=0;i<3;i++)
	cout<<"\t"<<"pos_d["<<i<<"] = "<<CURR_RAVEN_STATE.pos_d[i];
	cout<<endl<<endl;
	
	SHOW_SUB = false;
}


/**
*	\fn int getKey()
*
*	\brief gets keyboard character for switch case's of console_process()
*
* 	\param void
*
*	\return character int
*/
int getKey() 
{
    	int character;
    	struct termios orig_term_attr;
    	struct termios new_term_attr;

    	// set the terminal to raw mode 
    	tcgetattr(fileno(stdin), &orig_term_attr);
    	memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    	new_term_attr.c_lflag &= ~(ECHO|ICANON);
    	new_term_attr.c_cc[VTIME] = 0;
    	new_term_attr.c_cc[VMIN] = 0;
    	tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);

    	// read a character from the stdin stream without blocking 
    	//   returns EOF (-1) if no character is available 
    	character = fgetc(stdin);

   	// restore the original terminal attributes 
    	tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    	return character;
}


/**
*	\fn void outputSTATUS()
*
* 	\brief shows the AutoCircle generator status (display every one sec)
*
* 	\param void
*
*	\return void
*/
void outputSTATUS()
{
	//.. 
	//Make it better
        cout<<"RADIUS = "<<RADIUS<<endl;
	cout<<"SPEED = "<<SPEED<<endl<<endl;
}


/**
*	\fn void *console_process(void *)
*
* 	\brief this thread is dedicated to console io
*
* 	\param a pointer to void
*
*	\return void
*/
void *console_process(void*)
{
	if(ros::isInitialized())
	{
		int theKey;
		bool print_msg = true;
		ros::Time time;
		time = time.now();

		while(ros::ok() && !TERMINATED)
		{
			if(print_msg)
			{
				cout<<endl;
				cout<<"Auto Circle Generator Selection Menu:"<<endl;
				cout<<"----------------------------------------------------"<<endl;
				cout<<"\t'1' : Increase Circle Radius"<<endl;
				cout<<"\t'2' : Decrease Circle Radius"<<endl;
				cout<<"\t'3' : Increase Raven Moving Speed"<<endl;
				cout<<"\t'4' : Decrease Raven Moving Speed"<<endl;
				cout<<"\t'5' : Toggle circling direction"<<endl;
				cout<<"\t'6' : Toggle pause/resume "<<endl;
				cout<<"\t'7' : Toggle console messages"<<endl;
				cout<<"\t'8' : Quit"<<endl<<endl;
				print_msg = false;
			}

			theKey = getKey();

			switch(theKey)
			{
				case '1':
				{
					cout<<"You chose 1 : Increase Circle Radius."<<endl;
					RADIUS = (RADIUS+1>MAX_RADIUS) ? MAX_RADIUS : RADIUS+1;
					cout<<"\tnew RADIUS = "<<RADIUS<<endl;
					ComputeNewTrajectory();
					print_msg = true;
					break;
				}
				case '2':
				{
					cout<<"You chose 2 : Decrease Circle Radius."<<endl;
					RADIUS = (RADIUS-1<MIN_RADIUS) ? MIN_RADIUS : RADIUS-1;
					cout<<"\tnew RADIUS = "<<RADIUS<<endl;
					ComputeNewTrajectory();
					print_msg = true;
					break;
				}
				case '3':
				{
					cout<<"You chose 3 : Increase Raven Moving Speed."<<endl;
					SPEED = (SPEED+1>MAX_SPEED) ? MAX_SPEED : SPEED+1;
					cout<<"\tnew SPEED = "<<SPEED<<endl;
					print_msg = true;
					break;
				}
				case '4':
				{
					cout<<"You chose 4 : Decrease Raven Moving Speed."<<endl;
					SPEED = (SPEED-1<MIN_SPEED) ? MIN_SPEED : SPEED-1;
					cout<<"\tnew SPEED = "<<SPEED<<endl;
					print_msg = true;
					break;
				}
				case '5':
				{
					cout<<"You chose 5 : Toggle circling direction."<<endl;
					DIRECTION = DIRECTION*(-1);
					//.. comment may be wrong
					if(DIRECTION > 0)
						cout<<"Change to clockwise circle trajectory."<<endl;
					else
						cout<<"Change to counter-clockwise circle trajectory."<<endl;
					print_msg = true;
					break;
				}
				case '6':
				{
					cout<<"You chose 6 : Toggle pause/resume."<<endl;
					PAUSE = !PAUSE;
					if(PAUSE)
						cout<<"AutoCircling is paused."<<endl;
					else
						cout<<"AutoCircling is resumed."<<endl;
					print_msg = true;
					break;
				}
				case '7':
				{
					cout<<"You chose 7 : Toggle console messages."<<endl;
					SHOW_STATUS = !SHOW_STATUS;
					if(SHOW_STATUS)
						cout<<"Console message turned on."<<endl;
					else
						cout<<"Console message turned off."<<endl;
					print_msg = true;
					break;
				}
				case '8':
				{
					cout<<"You chose 8 : Quit."<<endl;
					cout<<"Now press ^C to terminate."<<endl<<endl;
					TERMINATED = true;
					print_msg = true;
					break;
				}			
			
			}

			if(SHOW_STATUS && (time.now()-time).toSec() > 1)
			{
				outputSTATUS();
				time = time.now();
			}
		}
		cout<<"console_process is shutdown."<<endl;
		return( NULL);
	}
	else
		return 0;
}


/**
*	\fn void *ros_process(void *)
*
* 	\brief this thread is dedicated to ros pub & sub
*
* 	\param a pointer to void
*
*	\return void
*/
void *ros_process(void*)
{
	if(ros::isInitialized())
	{
		static int PUB_INDEX = 0;
		static int PUB_COUNT = 0;

		while(ros::ok() && !TERMINATED)
		{
			if(!PAUSE)
			{
				publish_raven_automove_ros(PUB_INDEX,PUB_COUNT);

				PUB_INDEX = (PUB_INDEX + DIRECTION) % 360;
				PUB_COUNT ++;
			}
		}
		cout<<"ros_process is shutdown."<<endl;
		return( NULL);
	}
	else
		return 0;
}

/**
*	\fn void finalWords()
*
* 	\brief these are the final things to display
*
* 	\param void
*
*	\return void
*/
void finalWords()
{
	cout<<"Terminating the AutoCircle_generator." <<endl;
	cout<<"----------------------------------------------------"<<endl;
	cout<<"GoodBye!"<<endl<<endl<<endl;
}



