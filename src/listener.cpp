#include <string.h>
#include <pthread.h>
#include <ros/ros.h>
#include <ros/transport_hints.h>
#include <tf/transform_datatypes.h>
//#include "bullet/bullet3-2.83.7/src/LinearMath/btVector3.h"
//#include "bullet/bullet3-2.83.7/src/LinearMath/btTransform.h"
//#include "bullet/bullet3-2.83.7/src/LinearMath/btQuaternion.h"
#include "raven_2/raven_automove.h"
#include "raven_2/raven_state.h"
#include "DS1.h" //struct param_pass is defined here.

#define ROS_PUBLISH_RATE 100
#define LEFT_ARM 0  //is also the GOLD_ARM
#define RIGHT_ARM 1 //is also the GREEN_ARM

using namespace raven_2;
using namespace std;

//---------------------------global variables---------------------------
ros:: Subscriber sub_automove;
ros:: Publisher pub_ravenstate;

raven_2::raven_state CURR_RAVEN_STATE;

int PUB_COUNT;
int SUB_COUNT;
static struct param_pass data1;
tf::Quaternion Q_ori[2];

pthread_t rt_thread;

//-------------------------function declaration-------------------------
void autoincrCallback(raven_2::raven_automove);
int init_ravenstate_publishing(ros::NodeHandle &);
int init_ros(int, char**);
void init_sys(int, char**);
void init_raven();

void output_STATUS();

void publish_raven_state_ros();
void *rt_process(void*);

//---------------------------------main---------------------------------
int main(int argc, char **argv)
{
	init_sys(argc,argv);
	
	pthread_create(&rt_thread,NULL,rt_process,NULL);

	ros::spin();
	return 0;
}

//--------------------------function definition--------------------------
/**
*	\fn void *rt_process(void *)
*
* 	\brief this thread is dedicated to ros publishing of raven_state data
*
* 	\param a pointer to void
*
*	\return void
*/
void *rt_process(void*)
{
	if(ros::isInitialized())
	{
		ros::Time time;
		time = time.now();
		while(ros::ok())
		{
			publish_raven_state_ros();
			if((time.now()-time).toSec() > 1)
			{
				output_STATUS();
				time = time.now();
			}
		}
		cout<<"rt_process is shutdown."<<endl;
		return( NULL);
	}
	else
		return 0;
}



/**
*	\fn void autoincrCallback(raven_2::raven_automove msg)
*
*	\brief This function is automatically called whenever someone publish to the raven_automove topic
*
* 	\param raven_2::raven_automove msg
*
*	\return void
*/
void autoincrCallback(raven_2::raven_automove msg) //this was in local_io.cpp
{
	tf::Transform in_incr[2];

	tf::transformMsgToTF(msg.tf_incr[0], in_incr[0]);
	tf::transformMsgToTF(msg.tf_incr[1], in_incr[1]);

	for (int i=0;i<2;i++)
	{
		//add position increment
		tf::Vector3 tmpvec = in_incr[i].getOrigin();
      		data1.xd[i].x += int(tmpvec[0]);
      		data1.xd[i].y += int(tmpvec[1]);
		data1.xd[i].z += int(tmpvec[2]);

		//add rotation increment
		tf::Quaternion q_temp(in_incr[i].getRotation());
		if (q_temp != tf::Quaternion::getIdentity())
		{
			int armidx = 1; //just for simplicity for now

			Q_ori[armidx] = q_temp*Q_ori[armidx];
			tf::Matrix3x3 rot_mx_temp(Q_ori[armidx]);
			for (int j=0;j<3;j++)
			for (int k=0;k<3;k++)
				data1.rd[i].R[j][k] = rot_mx_temp[j][k];
			
		}
	}
	SUB_COUNT ++;
}


/**
*	\fn int init_ravenstate_publishing(ros::NodeHandle &n) 
*
*	\brief initialize the pub & sub relation.
*
* 	\param int argc, char** argv
*
*	\return int
*/
int init_ravenstate_publishing(ros::NodeHandle &n)  //this was in local_io.cpp
{
	//not include this part for now:
	//..

	pub_ravenstate = n.advertise<raven_state>("raven_state",1);
	sub_automove = n.subscribe<raven_automove>("raven_automove",1,autoincrCallback,ros::TransportHints().unreliable());

}


/**
*	\fn int init_sys(int argc, char** argv) 
*
*	\brief initialize ROS and connect the pub & sub relation.
*
* 	\param int argc, char** argv
*
*	\return void
*/
void init_sys(int argc, char** argv)
{
	init_raven();

	if(init_ros(argc,argv))
	{
		cerr<<"ERROR! Fail to init ROS. Exiting.\n";
		exit(1);
	}

	cout<<"Welcome to the Auto Circle Generator for RAVEN2"<<endl;
	cout<<"I'm waiting for the talker..."<<endl;
	 
}


/**
*	\fn int init_sys() 
*
*	\brief initialize ROS and connect the pub & sub relation.
*
* 	\param void
*
*	\return void
*/
void init_raven()
{
	Q_ori[0] =  tf::Quaternion::getIdentity(); //.. maybe unused
	Q_ori[1] =  tf::Quaternion::getIdentity();

	//Assume this is our raven init state
	//..
	data1.xd[LEFT_ARM].x = 1; 
	data1.xd[LEFT_ARM].y = 2;
	data1.xd[LEFT_ARM].z = 3;
	data1.xd[RIGHT_ARM].x = 4;
	data1.xd[RIGHT_ARM].y = 5;
	data1.xd[RIGHT_ARM].z = 6;

	for(int orii=0; orii<3;orii++)
	for(int orij=0; orij<3;orij++)
	{
		data1.rd[ LEFT_ARM].R[orii][orij] = orii*3+orij;
		data1.rd[RIGHT_ARM].R[orii][orij] = 8 + orii*3+orij;
	}
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
int init_ros(int argc, char** argv)  //this was in rt_process_preempt.cpp
{
	//initialize counter
	PUB_COUNT = 0;
	SUB_COUNT = 0;

	//initialize ros 
	ros::init(argc,argv,"r2_control",ros::init_options::NoSigintHandler);
	
	//establish ros pub & sub relation
	static ros::NodeHandle n;
	init_ravenstate_publishing(n);
	
	return 0;
}


/**
*	\fn int output_STATUS()
*
*	\brief Display the recieved data from the Auto Circle Generator.
*
* 	\param void
*
*	\return void
*/
void output_STATUS()
{
	cout<<endl<<endl;
	ROS_INFO("listenerAutoCircle publish: raven_state[%d]", PUB_COUNT);
	ROS_INFO("listenerAutoCircle subscribe: raven_automove[%d]", SUB_COUNT);

	for(int i=0; i<2; i++) 
	{
		//display position increment
		cout<<"\tdata1.xd["<<i<<"] = ";
		cout<<"("<<data1.xd[i].x<<","<<data1.xd[i].y<<","<<data1.xd[i].z<<")"<<endl;

		//display rotation increment
		cout<<"\tdata1.rd["<<i<<"] = ";
		for (int j=0;j<3;j++)
		{	
			cout<<endl<<"\t\t";
			for (int k=0;k<3;k++)
			{
				cout<<data1.rd[i].R[j][k]<<"\t";
			}
		}
		cout<<endl;
	}
}


/**
*	\fn void publish_raven_state_ros()
*
*	\brief publish the auto circle command to raven_automove topic on ROS.
*
* 	\param void
*
*	\return void
*/
void publish_raven_state_ros()
{
	static ros::Rate loop_rate(ROS_PUBLISH_RATE);
	static raven_state msg_raven_state;

	//prepare "msg_raven_state" for publishing
	msg_raven_state.pos_d[0] = data1.xd[LEFT_ARM].x;
	msg_raven_state.pos_d[1] = data1.xd[LEFT_ARM].y;
	msg_raven_state.pos_d[2] = data1.xd[LEFT_ARM].z;
	msg_raven_state.pos_d[3] = data1.xd[RIGHT_ARM].x;
	msg_raven_state.pos_d[4] = data1.xd[RIGHT_ARM].y;
	msg_raven_state.pos_d[5] = data1.xd[RIGHT_ARM].z;

	msg_raven_state.pos[0] = data1.xd[LEFT_ARM].x; //.. actual position (should get that somehow)
	msg_raven_state.pos[1] = data1.xd[LEFT_ARM].y;
	msg_raven_state.pos[2] = data1.xd[LEFT_ARM].z;
	msg_raven_state.pos[3] = data1.xd[RIGHT_ARM].x;
	msg_raven_state.pos[4] = data1.xd[RIGHT_ARM].y;
	msg_raven_state.pos[5] = data1.xd[RIGHT_ARM].z;

	for(int orii=0; orii<3;orii++)
	for(int orij=0; orij<3;orij++)
	{
		msg_raven_state.ori[ LEFT_ARM*9+orii*3+orij] = data1.rd[ LEFT_ARM].R[orii][orij];
		msg_raven_state.ori[RIGHT_ARM*9+orii*3+orij] = data1.rd[RIGHT_ARM].R[orii][orij];
		
		//.. actual rotation (should get that somehow)
		msg_raven_state.ori_d[ LEFT_ARM*9+orii*3+orij] = data1.rd[ LEFT_ARM].R[orii][orij];
		msg_raven_state.ori_d[RIGHT_ARM*9+orii*3+orij] = data1.rd[RIGHT_ARM].R[orii][orij];
	}

	pub_ravenstate.publish(msg_raven_state);
	
	ros::spinOnce();
	

	loop_rate.sleep();
	PUB_COUNT ++;
}
