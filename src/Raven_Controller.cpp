#include "Raven_Controller.h"


/**
*	\fn Raven_Controller()
*
* 	\brief this is the constructor
*
* 	\param void
*
*	\return none
*/
Raven_Controller::Raven_Controller()
{

}




/**
*	\fn initial()
*
* 	\brief initialize everything for this program.
*
* 	\param int argc, char** argv
*
*	\return void
*/
void Raven_Controller::initial(int argc, char** argv)
{
	init_sys();
	if(!init_ros(argc,argv))
  	{
     		ROS_ERROR("Fail to initialize ROS. Exiting!");
		exit(1);
  	}
	
	init_pathplanner();
	//print the welcome greets on console
	init_words();
}



/**
*	\fn void init_sys() 
*
*	\brief initialize default system parameter settings.
*
* 	\param void
*
*	\return void
*/
void Raven_Controller::init_sys()  
{
	this->RADIUS = 1;
	this->SPEED = 1;
	this->DIRECTION = 1;
	this-> BASE_PLANE =  YZ_PLANE;

	this->PUB_COUNT = 0;
	this->SUB_COUNT = 0;

	this->RECEIVED_FIRST = false;
	this->SHOW_STATUS = false;
	this->PAUSE = false;
	
}



/**
*	\fn int init_ros(int argc, char** argv) 
*
*	\brief initialize ROS and connect the pub & sub relation.
*
* 	\param int argc, char** argv
*
*	\return bool
*/
bool Raven_Controller::init_ros(int argc, char** argv) 
{
	//initialize ROS
	ros::init(argc, argv, "AutoCircle_generator");

	static ros::NodeHandle n;
  	RavenAutomove_publisher = n.advertise<raven_automove>("raven_automove", 1);
	RavenState_subscriber   = n.subscribe("ravenstate",1,&Raven_Controller::callback_raven_state,this);

	return true;
}



/**
*	\fn void init_pathplanner) 
*
*	\brief initialize objects from Raven_PathPlanner class.
*
* 	\param void
*
*	\return void
*/
void Raven_Controller::init_pathplanner()  
{
	if(!LEFT_PATH.set_ArmType(LEFT_ARM) || !RIGHT_PATH.set_ArmType(RIGHT_ARM))
	{
		ROS_ERROR("Fail to set RAVEN arm type. Exiting!");
		exit(1);
	}
	if(!LEFT_PATH.set_Radius(RADIUS) || !RIGHT_PATH.set_Radius(RADIUS))
	{
		ROS_ERROR("Fail to set circle radius. Exiting!");
		exit(1);
	}
	if(!LEFT_PATH.set_Speed(SPEED) || !RIGHT_PATH.set_Speed(SPEED))
	{
		ROS_ERROR("Fail to set circle speed. Exiting!");
		exit(1);
	}
	if(!LEFT_PATH.set_Direction(DIRECTION) || !RIGHT_PATH.set_Direction(DIRECTION))
	{
		ROS_ERROR("Fail to set circle direction. Exiting!");
		exit(1);
	}
	if(!LEFT_PATH.set_BasePlane(BASE_PLANE) || !RIGHT_PATH.set_BasePlane(BASE_PLANE))
	{
		ROS_ERROR("Fail to set circle base plane. Exiting!");
		exit(1);
	}

}



/**
*	\fn void init_words() 
*
*	\brief show greeting words on console.
*
* 	\param void
*
*	\return void
*/
void Raven_Controller::init_words()  
{
	string start = "0";
	do
	{
		cout<<endl<<endl;
		cout<<"Welcome to the Auto Circle Generator for RAVEN2"<<endl<<endl;
		cout<<"Default settings: RADIUS = "<<RADIUS<<endl;
		cout<<"                  SPEED = "<<SPEED<<endl;
		cout<<"                  DIRECTION = "<<DIRECTION<<endl;
		cout<<"                  BASE = Y-Z plane"<<endl<<endl;
		cout<<"Please press \"Enter\" to start!";
		cin.clear();
		getline(std::cin,start);
	}while(start!="");

	cout<<"Auto Circle Generator starting..."<<endl;
}



/**
*	\fn void menu_words() 
*
*	\brief show menu words on console.
*
* 	\param void
*
*	\return void
*/
bool Raven_Controller::menu_words(bool print_menu)  
{
	if(print_menu)
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
		cout<<"\t'8' : Set as Circle Center."<<endl;
		cout<<"\t'9' : Change Circle Base Plane."<<endl;
		cout<<"\t'^C': Quit"<<endl<<endl;
	}
	return false;
}



/**
*	\fn void final_words() 
*
*	\brief show goodbye words on console.
*
* 	\param void
*
*	\return void
*/
void Raven_Controller::final_words()  
{
	cout<<"Terminating the AutoCircle_generator." <<endl;
	cout<<"----------------------------------------------------"<<endl;
	cout<<"GoodBye!"<<endl<<endl<<endl;
}




/**
*	\fn void start_thread()
*
* 	\brief start the console_thread and ros_thread
*
* 	\param void
*
*	\return void
*/
void Raven_Controller::start_thread()
{
	pthread_create(&console_thread,NULL,Raven_Controller::static_console_process,this);
	pthread_create(&ros_thread,NULL,Raven_Controller::static_ros_process,this);

}



/**
*	\fn void join_thread()
*
* 	\brief join the console_thread and ros_thread
*
* 	\param void
*
*	\return void
*/
void Raven_Controller::join_thread()
{
	pthread_join(console_thread,NULL);
	pthread_join(ros_thread,NULL);

	final_words();
}



/**
*	\fn void *console_process(void)
*
* 	\brief this thread is dedicated to console io
*
* 	\param a pointer to void
*
*	\return void
*/
void* Raven_Controller::console_process(void)
{
	if(ros::isInitialized())
	{
		int theKey;
		bool print_menu = true;
		ros::Time time;
		time = time.now();

		while(ros::ok())
		{
			print_menu = menu_words(print_menu);
			theKey = getKey();

			switch(theKey)
			{
				case '1':
				{
					RADIUS = (RADIUS+1 > MAX_RADIUS) ? MAX_RADIUS : RADIUS+1;
					LEFT_PATH.set_Radius(RADIUS);
					RIGHT_PATH.set_Radius(RADIUS);

					cout<<"You chose 1 : Increase Circle Radius."<<endl;
					cout<<"\tnew RADIUS = "<<RADIUS<<endl;
					print_menu = true;
					break;
				}
				case '2':
				{
					RADIUS = (RADIUS-1 < MIN_RADIUS) ? MIN_RADIUS : RADIUS-1;
					LEFT_PATH.set_Radius(RADIUS);
					RIGHT_PATH.set_Radius(RADIUS);

					// [DANGER]: related to modification parameter tuning
					// observation from experiments
					if(RADIUS < SMALL_RADIUS)	
						SPEED = (SPEED > SMALL_RADIUS_MAX_SPEED) ? SMALL_RADIUS_MAX_SPEED : SPEED;

					LEFT_PATH.set_Speed(SPEED);
					RIGHT_PATH.set_Speed(SPEED);

					cout<<"You chose 2 : Decrease Circle Radius."<<endl;
					cout<<"\tnew RADIUS = "<<RADIUS<<endl;
					print_menu = true;
					break;
				}
				case '3':
				{
					SPEED = (SPEED+1 > MAX_SPEED) ? MAX_SPEED : SPEED+1;

					// [DANGER]: related to modification parameter tuning
					// observation from experiments
					if(BASE_PLANE != YZ_PLANE)
						SPEED = (SPEED > VERTICLE_CIRCLE_MAX_SPEED) ? VERTICLE_CIRCLE_MAX_SPEED : SPEED;
					if(RADIUS < SMALL_RADIUS)	
						SPEED = (SPEED > SMALL_RADIUS_MAX_SPEED) ? SMALL_RADIUS_MAX_SPEED : SPEED;

					LEFT_PATH.set_Speed(SPEED);
					RIGHT_PATH.set_Speed(SPEED);

					cout<<"You chose 3 : Increase Raven Moving Speed."<<endl;
					cout<<"\tnew SPEED = "<<SPEED<<endl;
					print_menu = true;
					break;
				}
				case '4':
				{
					SPEED = (SPEED-1 < MIN_SPEED) ? MIN_SPEED : SPEED-1;
					LEFT_PATH.set_Speed(SPEED);
					RIGHT_PATH.set_Speed(SPEED);

					cout<<"You chose 4 : Decrease Raven Moving Speed."<<endl;
					cout<<"\tnew SPEED = "<<SPEED<<endl;
					print_menu = true;
					break;
				}
				case '5':
				{

					DIRECTION = DIRECTION*(-1);
					LEFT_PATH.set_Direction(DIRECTION);
					RIGHT_PATH.set_Direction(DIRECTION);

					cout<<"You chose 5 : Toggle circling direction."<<endl;
					if(DIRECTION > 0)
						cout<<"\tChange to counter-clockwise circle trajectory."<<endl;
					else
						cout<<"\tChange to clockwise circle trajectory."<<endl;
					print_menu = true;
					break;
				}
				case '6':
				{
					PAUSE = !PAUSE;

					cout<<"You chose 6 : Toggle pause/resume."<<endl;
					if(PAUSE)
						cout<<"\tAutoCircling is paused."<<endl;
					else
						cout<<"\tAutoCircling is resumed."<<endl;
					print_menu = true;
					break;
				}
				case '7':
				{
					SHOW_STATUS = !SHOW_STATUS;

					cout<<"You chose 7 : Toggle console messages."<<endl;
					if(SHOW_STATUS)
						cout<<"\tConsole message turned on."<<endl;
					else
						cout<<"\tConsole message turned off."<<endl;
					print_menu = true;
					break;
				}		
				case '8':
				{
					LEFT_PATH.set_Center(CURR_RAVEN_STATE.pos);
					RIGHT_PATH.set_Center(CURR_RAVEN_STATE.pos);

					cout<<"You chose 8 : Set as Circle Center."<<endl;
					cout<<"\tnew Center[LEFT]: X,Y,Z = ("<<CURR_RAVEN_STATE.pos[0]<<",";
					cout<<CURR_RAVEN_STATE.pos[1]<<","<<CURR_RAVEN_STATE.pos[2]<<")"<<endl;
					cout<<"\tnew Center[RIGHT]: X,Y,Z = ("<<CURR_RAVEN_STATE.pos[3]<<",";
					cout<<CURR_RAVEN_STATE.pos[4]<<","<<CURR_RAVEN_STATE.pos[5]<<")"<<endl;
					print_menu = true;
					break;
				}
				case '9':
				{
					LEFT_PATH.set_Center(CURR_RAVEN_STATE.pos);   // important step to avoid confusion.
					RIGHT_PATH.set_Center(CURR_RAVEN_STATE.pos);

					BASE_PLANE = (BASE_PLANE + 1) % 3;
					LEFT_PATH.set_BasePlane(BASE_PLANE);
					RIGHT_PATH.set_BasePlane(BASE_PLANE);

					SPEED = (SPEED > CHANGE_BASEPLANE_MAX_SPEED) ? CHANGE_BASEPLANE_MAX_SPEED : SPEED;
					LEFT_PATH.set_Speed(SPEED);
					RIGHT_PATH.set_Speed(SPEED);

					cout<<"You chose 9 : Change Circle Base Plane."<<endl;
					if(BASE_PLANE == YZ_PLANE)
						cout<<"\tChange to Y-Z plane."<<endl; // default setting
					else if(BASE_PLANE == XZ_PLANE)
						cout<<"\tChange to X-Z plane."<<endl;
					else
						cout<<"\tChange to X-Y plane."<<endl;
					print_menu = true;
					break;
				}
				/*
				// [DANGER]: for modification parameter tuning only
				case 'a':
				{	cout<<"You chose a : increase Modi_Scale."<<endl;
					LEFT_PATH.set_Modi_Scale(1);
					print_menu = true;
					break;
				}
				case 'z':
				{	cout<<"You chose z : decrease Modi_Scale."<<endl;
					LEFT_PATH.set_Modi_Scale(-1);
					print_menu = true;
					break;
				}
				case 's':
				{	cout<<"You chose s : increase Modi_Speed_Pow."<<endl;
					LEFT_PATH.set_Modi_Speed_Pow(1);
					print_menu = true;
					break;
				}
				case 'x':
				{	cout<<"You chose x : decrease Modi_Speed_Pow."<<endl;
					LEFT_PATH.set_Modi_Speed_Pow(-1);
					print_menu = true;
					break;
				}
				case 'd':
				{	cout<<"You chose d : increase Modi_Dista_Pow."<<endl;
					LEFT_PATH.set_Modi_Dista_Pow(1);
					print_menu = true;
					break;
				}
				case 'c':
				{	cout<<"You chose c : decrease Modi_Dista_Pow."<<endl;
					LEFT_PATH.set_Modi_Dista_Pow(-1);
					print_menu = true;
					break;
				}
				*/
	
			}

			if(SHOW_STATUS && (time.now()-time).toSec() > 1)
			{
				output_STATUS();
				time = time.now();
			}
		}
		cout<<"console_process is shutdown."<<endl;

	}
	return 0;
}



/**
*	\fn void *ros_process(void)
*
* 	\brief this thread is dedicated to ros pub & sub
*
* 	\param a pointer to void
*
*	\return void
*/
void* Raven_Controller::ros_process(void)
{
	if(ros::isInitialized())
	{
		if(!RECEIVED_FIRST)
			cout<<endl<<"Waiting for the first receive of raven_state..."<<endl;
		else
		{
			cout<<"First raven_state received."<<endl;
			cout<<"Save as default center potition."<<endl;
	
			LEFT_PATH.set_Center(CURR_RAVEN_STATE.pos);
			RIGHT_PATH.set_Center(CURR_RAVEN_STATE.pos);
		}

		while(ros::ok() && RECEIVED_FIRST)
		{
			// (1) update current raven_state (for future computation uses)
			LEFT_PATH.set_Current_Pos(CURR_RAVEN_STATE.pos);
			LEFT_PATH.set_Current_Ori(CURR_RAVEN_STATE.ori);

			RIGHT_PATH.set_Current_Pos(CURR_RAVEN_STATE.pos);
			RIGHT_PATH.set_Current_Ori(CURR_RAVEN_STATE.ori);

			// (2) generate new command (plan trajectory)
			if(PAUSE) 
			{
				// stop raven
				TF_INCR[LEFT_ARM] = LEFT_PATH.ComputeNullTrajectory(); 
				TF_INCR[RIGHT_ARM] = RIGHT_PATH.ComputeNullTrajectory(); 
			}
			else 
			{
				// normal moving case
				//TF_INCR[LEFT_ARM] = LEFT_PATH.ComputeNullTrajectory(); 
				TF_INCR[LEFT_ARM] = LEFT_PATH.ComputeCircleTrajectory();
				TF_INCR[RIGHT_ARM] = RIGHT_PATH.ComputeCircleTrajectory();

			}

			// (3) publish new command (send it out)
			publish_raven_automove();

		}

		if(RECEIVED_FIRST)
			cout<<"ros_process is shutdown."<<endl;
	}
	return 0;
}



void * Raven_Controller::static_console_process(void* classRef)
{
	return ((Raven_Controller *)classRef)->console_process();
}



void * Raven_Controller::static_ros_process(void* classRef)
{
	return ((Raven_Controller *)classRef)->ros_process();
}



/**
*	\fn void publish_raven_automove()
*
*	\brief publish the auto circle command to raven_automove topic on ROS.
*
* 	\param int
*
*	\return void
*/
void Raven_Controller::publish_raven_automove()
{
	static ros::Rate loop_rate(ROS_PUBLISH_RATE);
	static raven_automove msg_raven_automove;	

	// (1) wrap up the new command	
	msg_raven_automove.hdr.stamp = msg_raven_automove.hdr.stamp.now(); //hdr

	tf::transformTFToMsg(TF_INCR[LEFT_ARM], msg_raven_automove.tf_incr[LEFT_ARM]);   //tf_incr
	tf::transformTFToMsg(TF_INCR[RIGHT_ARM], msg_raven_automove.tf_incr[RIGHT_ARM]);

	// (2) send new command
	RavenAutomove_publisher.publish(msg_raven_automove);
	ros::spinOnce();

	//(3) prepare for next publish
	loop_rate.sleep();
	PUB_COUNT ++;
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
void Raven_Controller::callback_raven_state(raven_2::raven_state msg) 
{
	// (1) save the updated raven_state 
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
	
	for(int i=0; i<18; i++)
	{
		CURR_RAVEN_STATE.ori[i] = msg.ori[i];
		CURR_RAVEN_STATE.ori_d[i] = msg.ori_d[i];
	}

	// (2) update recieved data count
	SUB_COUNT ++;
	RECEIVED_FIRST = true;
}


// pointer version of callback function (not working for now)
//..

/**
*	\fn void autoRavenStateCallback(boost::shared_ptr< ::raven_state_<ContainerAllocator> const> msg) 
*
*	\brief This function is automatically called whenever someone publish to raven_state topic
*
* 	\param boost::shared_ptr< ::raven_state_<ContainerAllocator> const
*
*	\return void
*/
/*
void autoRavenStateCallback(boost::shared_ptr< ::raven_state_<ContainerAllocator> const> msg) 
{
	// (1) save the updated raven_state 
	CURR_RAVEN_STATE.runlevel = msg->runlevel;
	CURR_RAVEN_STATE.sublevel = msg->sublevel;
	CURR_RAVEN_STATE.last_seq = msg->last_seq;
	CURR_RAVEN_STATE.dt = msg->dt;

	for(int i=0; i<2; i++)
	{
		CURR_RAVEN_STATE.type[i] = msg->type[i];
		CURR_RAVEN_STATE.grasp_d[i] = msg->grasp_d[i];
	}

	for(int i=0; i<6; i++)
	{
		CURR_RAVEN_STATE.pos[i] = msg->pos[i];
		CURR_RAVEN_STATE.pos_d[i] = msg->pos_d[i];
	}


	for(int i=0; i<18; i++)
	{
		CURR_RAVEN_STATE.ori[i] = msg->ori[i];
		CURR_RAVEN_STATE.ori_d[i] = msg->ori_d[i];
	}

	// (2) update recieved data count
	SUB_COUNT ++;
}
*/



/**
*	\fn void output_fSTATUS()
*
* 	\brief shows the AutoCircle generator status (display every one sec)
*
* 	\param void
*
*	\return void
*/
void Raven_Controller::output_STATUS()
{
	tfScalar R = LEFT_PATH.get_Radius(); 		// in cm
	tfScalar dR = LEFT_PATH.get_Radius_Range();	// in cm
	tfScalar SP = LEFT_PATH.get_Speed(); 		// in cm/sec
	tfScalar k = LEFT_PATH.get_K();

        cout<<"current AutoCircle status :"<<endl;

	switch(BASE_PLANE)
	{	
		case YZ_PLANE:
			cout<<"\tBASE   = Y-Z plane";
		break;
		case XZ_PLANE:
			cout<<"\tBASE   = X-Z plane";
		break;
		case XY_PLANE:
			cout<<"\tBASE   = X-Y plane";
		break;
	}
	
	if(DIRECTION > 0)
		cout<<"\t(Counter-Clockwise circle)"<<endl;
	else
		cout<<"\t(Clockwise circle)"<<endl;

	cout<<"\tRADIUS = "<<R<<"~"<<(R+dR)<<" cm \t(level "<<RADIUS<<")"<<endl;
	cout<<"\tSPEED  = "<<SP<<" cm/sec\t(level "<<SPEED<<")"<<endl;
	cout<<"\tK      = "<<k<<" \t(Regulating Term: 0~1)"<<endl<<endl;

	/*
	// [DANGER]: for modification parameter tuning only
	tfScalar modi_scale = LEFT_PATH.get_Modi_Scale();
	tfScalar modi_speed_pow = LEFT_PATH.get_Modi_Speed_Pow();
	tfScalar modi_dista_pow = LEFT_PATH.get_Modi_Dista_Pow();
	
	cout<<"\tscale="<<modi_scale<<" speedPow="<<modi_speed_pow<<" disPow="<<modi_dista_pow<<endl;
	*/

	output_PATHinfo();
	output_PUBinfo();
	output_SUBinfo();
}



/**
*	\fn void output_PATHinfo()
*
* 	\brief shows the publish status (display every one sec)
*
* 	\param void
*
*	\return void
*/
void Raven_Controller::output_PATHinfo()
{
	cout<<"current PathPlanner status : "<<endl;
	
	LEFT_PATH.show_delPos();
	//RIGHT_PATH.show_delPos(); 	//(RIGHT_ARM unused right now)

	LEFT_PATH.show_Center();
	//RIGHT_PATH.show_Center(); 	//(RIGHT_ARM unused right now)
	
	LEFT_PATH.show_Distance();
	//RIGHT_PATH.show_Distance(); 	//(RIGHT_ARM unused right now)

	LEFT_PATH.show_PathState();
	//RIGHT_PATH.show_PathState(); 	//(RIGHT_ARM unused right now)
	cout<<endl<<endl;
		
}




/**
*	\fn void output_PUBinfo()
*
* 	\brief shows the publish status (display every one sec)
*
* 	\param void
*
*	\return void
*/
void Raven_Controller::output_PUBinfo()
{
	ROS_INFO("talkerAutoCircle publish: raven_automove[%d]", PUB_COUNT);
}



/**
*	\fn void output_SUBinfo()
*
* 	\brief shows the subscribe status (display every one sec)
*
* 	\param void
*
*	\return void
*/
void Raven_Controller::output_SUBinfo()
{
	ROS_INFO("talkerAutoCircle subscribe: raven_state[%d]", SUB_COUNT);

	//raven position
	cout<<"\t"<<"pos[LEFT] = ("<<CURR_RAVEN_STATE.pos[0]<<","<<CURR_RAVEN_STATE.pos[1];
	cout<<","<<CURR_RAVEN_STATE.pos[2]<<")"<<endl;
	
	//cout<<"\t"<<"pos[RIGHT] = ("<<CURR_RAVEN_STATE.pos[3]<<","<<CURR_RAVEN_STATE.pos[4];
	//cout<<","<<CURR_RAVEN_STATE.pos[5]<<")"<<endl;

	// raven rotation
	//cout<<"\t"<<"ori[LEFT] = \t\t\tori[RIGHT] = "<<endl;
	cout<<"\t"<<"ori[LEFT] = "<<endl;
	for(int orii=0; orii<3; orii++)
	{
		cout<<"\t\t";
		for(int orij=0; orij<3; orij++)
		{
			cout<<CURR_RAVEN_STATE.ori[LEFT_ARM*9+orii*3+orij]<<"\t";
		}
		//cout<<"\t";
		//for(int orij=0; orij<3; orij++)
		//{
		//	cout<<CURR_RAVEN_STATE.ori[RIGHT_ARM*9+orii*3+orij]<<"\t";
		//}
		cout<<endl;
	}
	cout<<endl;

	/* 
	// raven desired rotation (not important)
	cout<<"\t"<<"ori_d[LEFT_ARM] = \t\tori_d[RIGHT_ARM] = "<<endl;
	for(int orii=0; orii<3; orii++)
	{
		cout<<"\t\t";
		for(int orij=0; orij<3; orij++)
		{
			cout<<CURR_RAVEN_STATE.ori_d[LEFT_ARM*9+orii*3+orij]<<"\t";
		}
		cout<<"\t";
		for(int orij=0; orij<3; orij++)
		{
			cout<<CURR_RAVEN_STATE.ori_d[RIGHT_ARM*9+orii*3+orij]<<"\t";
		}
		cout<<endl;
	}
	cout<<endl
	*/

	cout<<endl;
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
int Raven_Controller::getKey() 
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
