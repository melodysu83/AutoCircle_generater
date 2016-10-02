#include "Raven_PathPlanner.h"

/**
*	\fn Raven_PathPlanner()
*
* 	\brief this is the constructor
*
* 	\param void
*
*	\return none
*/
Raven_PathPlanner::Raven_PathPlanner()
{
	Modi_Scale =     DEFAULT_MODIFICATION_SCALE;
	Modi_Speed_Pow = DEFAULT_MODIFICATION_SPEED_POWER;
	Modi_Dista_Pow = DEFAULT_MODIFICATION_DISTANCE_POWER;

	X_AXIS.setValue(1,0,0);
	Y_AXIS.setValue(0,1,0);
	Z_AXIS.setValue(0,0,1);

	pthread_mutexattr_init(&data1MutexAttr);
	pthread_mutexattr_setprotocol(&data1MutexAttr,PTHREAD_PRIO_INHERIT);
	pthread_mutex_init(&data1Mutex,&data1MutexAttr);

}


/**
*	\fn bool set_Radius(int radius)
*
* 	\brief stores circular motion radius
*
* 	\param int
*
*	\return bool
*/
bool Raven_PathPlanner::set_Radius(int radius)
{
	if(radius > MAX_RADIUS || radius < MIN_RADIUS)
	{	
		ROS_ERROR("Invalid Radius value. Setting fail!");
		return false;
	}
	Radius = radius * RADIUS_level_TO_microm; // in micro meter

	return true;
}



/**
*	\fn bool set_Speed(int speed)
*
* 	\brief stores circular motion speed
*
* 	\param int
*
*	\return bool
*/
bool Raven_PathPlanner::set_Speed(int speed)
{
	if(speed > MAX_SPEED || speed < MIN_SPEED)
	{	
		ROS_ERROR("Invalid Speed value. Setting fail!");
		return false;
	}
	Speed = speed * DEL_POS_THRESHOLD / MAX_SPEED; // in micro meter

	//..
	// set proportional gain Kp (Depends on Speed value.)
	Kp = Modi_Scale * tfPow(Speed,Modi_Speed_Pow);
	
	return true;
}



/**
*	\fn bool set_Direction(int direc)
*
* 	\brief stores circle trajectory direction
*
* 	\param int
*
*	\return bool
*/
bool Raven_PathPlanner::set_Direction(int direc)
{
	if(direc != 1 && direc != -1)
	{	
		ROS_ERROR("Invalid Direction value. Setting fail!");
		return false;
	}
	Direction = direc;
	return true;
}



/**
*	\fn bool set_BasePlane(int plane)
*
* 	\brief stores the circular trajectory base plane
*
* 	\param int
*
*	\return bool
*/
bool Raven_PathPlanner::set_BasePlane(int plane)
{
	if(plane == YZ_PLANE || plane == XZ_PLANE || plane == XY_PLANE )
	{	
		Base_Plane = plane;
		return true;
	}
	ROS_ERROR("Invalid Direction value. Setting fail!");
	return false;
}



/**
*	\fn bool set_ArmType(int armtype)
*
* 	\brief stores RAVEN arm type
*
* 	\param int
*
*	\return bool
*/
bool Raven_PathPlanner::set_ArmType(int armtype)
{
	bool armOK = armtype == LEFT_ARM || armtype == RIGHT_ARM;
	ArmType = armtype;

	return armOK;
}



/**
*	\fn bool set_Center(boost::array<int, 6> center)
*
* 	\brief stores the circle center point
*
* 	\param boost::array<int, 6>
*
*	\return bool
*/
bool Raven_PathPlanner::set_Center(boost::array<int, 6> center)
{
	tfScalar X,Y,Z;

	// (1) store center postion value
	if(ArmType == LEFT_ARM)
	{
		X = center[0];
		Y = center[1];
		Z = center[2];
	}
	else if(ArmType == RIGHT_ARM)
	{
		X = center[3];
		Y = center[4];
		Z = center[5];
	}
	else //weird case
	{
		return false;
	}

	Center.setValue(X,Y,Z);

	// (2) update distance between current pos and center pos 
	Distance = DistanceOf(Center,Current_Pos); 
	checkPathState();

	return true;
}



/**
*	\fn bool set_Current_Pos(boost::array<int, 6> currpos)
*
* 	\brief stores the current position received from raven_state
*
* 	\param boost::array<int, 6>
*
*	\return bool
*/
bool Raven_PathPlanner::set_Current_Pos(boost::array<int, 6> currpos)
{
	tfScalar X,Y,Z;

	// (1) store current postion value
	if(ArmType == LEFT_ARM)
	{
		X = currpos[0];
		Y = currpos[1];
		Z = currpos[2];
	}
	else if(ArmType == RIGHT_ARM)
	{
		X = currpos[3];
		Y = currpos[4];
		Z = currpos[5];
	}
	else //weird case
	{
		return false;
	}

	Current_Pos.setValue(X,Y,Z);
	
	// (2) update distance between current pos and center pos 
	Distance = DistanceOf(Center,Current_Pos); 
	checkPathState();

	return true;
}



/**
*	\fn bool set_Current_Ori(boost::array<float, 18> rot_matix)
*
* 	\brief stores the current orientation received from raven_state
*
* 	\param boost::array<float, 18>
*
*	\return bool
*/
bool Raven_PathPlanner::set_Current_Ori(boost::array<float, 18> rot_matix)
{
	tfScalar m00, m01, m02, m10, m11, m12, m20, m21, m22;
	tfScalar qw,qx,qy,qz,tr;

	if(ArmType == LEFT_ARM)
	{
		m00 = rot_matix[0];
		m01 = rot_matix[1];
		m02 = rot_matix[2];
		m10 = rot_matix[3];
		m11 = rot_matix[4];
		m12 = rot_matix[5];
		m20 = rot_matix[6];
		m21 = rot_matix[7];
		m22 = rot_matix[8];
	}
	else if(ArmType == RIGHT_ARM)
	{
		m00 = rot_matix[9];
		m01 = rot_matix[10];
		m02 = rot_matix[11];
		m10 = rot_matix[12];
		m11 = rot_matix[13];
		m12 = rot_matix[14];
		m20 = rot_matix[15];
		m21 = rot_matix[16];
		m22 = rot_matix[17];
	}	
	else // weird case
	{
		return false;
	}

	tr = m00 + m11 + m22;

	if (tr > 0) 
	{ 
		  tfScalar S = sqrt(tr+1.0) * 2; // S=4*qw 
		  qw = 0.25 * S;
		  qx = (m21 - m12) / S;
		  qy = (m02 - m20) / S; 
		  qz = (m10 - m01) / S; 
	} 
	else if ((m00 > m11)&(m00 > m22)) 
	{ 
		  tfScalar S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx 
		  qw = (m21 - m12) / S;
		  qx = 0.25 * S;
		  qy = (m01 + m10) / S; 
		  qz = (m02 + m20) / S; 
	} 
	else if (m11 > m22) 
	{ 
		  tfScalar S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
		  qw = (m02 - m20) / S;
		  qx = (m01 + m10) / S; 
		  qy = 0.25 * S;
		  qz = (m12 + m21) / S; 
	} 
	else 
	{ 
		  tfScalar S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
		  qw = (m10 - m01) / S;
		  qx = (m02 + m20) / S;
		  qy = (m12 + m21) / S;
		  qz = 0.25 * S;
	}

		
	tf::Quaternion q_temp(qx,qy,qz,qw);

	return true;
}



/**
*	\fn tfScalar sign(tfScalar value)
*
* 	\brief this function returens the sign of the value
*
* 	\param tfScalar
*
*	\return tfScalar
*/
tfScalar Raven_PathPlanner::sign(tfScalar value)
{
	if(value >= 0)
		return 1;
	else 
		return -1;
}



/**
*	\fn void checkPathState()
*
* 	\brief this function checks whether RAVEN is in AROUND_CIRCLE or MOVETO_CIRCLE state
*	       AROUND_CIRCLE : occurs when RAVEN Current_Pos is in-orbit and follows the circle.
*              MOVETO_CIRCLE : occurs when RAVEN is finding its way back to the orbit.
*
* 	\param void
*
*	\return void
*/
void Raven_PathPlanner::checkPathState()
{
	PathState = AROUND_CIRCLE;
	
	Error = abs(Radius-Distance);

	if(Distance == 0 || Error > STATE_THRESHOLD)
		PathState = MOVETO_CIRCLE;	
}



/**
*	\fn tfScalar get_Radius()
*
* 	\brief returns the current radius (in cm)
*
* 	\param void
*
*	\return tfScalar
*/
tfScalar Raven_PathPlanner::get_Radius()
{
	tfScalar R = Radius/10000; // in cm
	R = 2*R/3 + 1;
/*
	// [DANGER]: related to modification parameter tuning
		     observation from experiments
		     relation from desired radius to actual radius!
		     here is the relation:
		     
	           radius level      desired radius     actual radius       
	     --------------------------------------------------------------
	               1                0.3 cm           1.2 ~ 1.3 cm
	               2                0.6 cm           1.4 ~ 1.5 cm
	               3                0.9 cm           1.6 ~ 1.7 cm
	               4                1.2 cm           1.8 ~ 1.9 cm
	               5                1.5 cm           2.0 ~ 2.1 cm
	               6                1.8 cm           2.2 ~ 2.4 cm
*/
	return R;
}



/**
*	\fn tfScalar get_Radius_Range()
*
* 	\brief returns the current radius range
*
* 	\param void
*
*	\return tfScalar
*/
tfScalar Raven_PathPlanner::get_Radius_Range()
{
	// [DANGER]: related to modification parameter tuning
	tfScalar result = 0.1;     // in cm
	tfScalar R = Radius/10000; // in cm

	if(R == 1.8)
	result = 0.2;  // observation from experiments
		       // how much does actual radius swings!
	
	return result;
}



/**
*	\fn tfScalar get_Speed()
*
* 	\brief returns the current Speed (in cm/sec)
*
* 	\param void
*
*	\return tfScalar
*/
tfScalar Raven_PathPlanner::get_Speed()
{
	tfScalar SP = Speed*ROS_PUBLISH_RATE/10000; // in cm/sec
	return SP;
}



/**
*	\fn tfScalar get_K()
*
* 	\brief stores the current position received from raven_state
*
* 	\param void
*
*	\return tfScalar
*/
tfScalar Raven_PathPlanner::get_K()
{
	tfScalar result = K;
	return result;
}



/**
*	\fn void show_PathState()
*
* 	\brief displays PathState value to console
*
* 	\param void
*
*	\return void
*/
void Raven_PathPlanner::show_PathState()
{
	if(ArmType == LEFT_ARM)
	{
		if(PathState == MOVETO_CIRCLE)
  			cout<<"\tPathState[LEFT] = MOVETO_CIRCLE"<<endl;

		else if(PathState == AROUND_CIRCLE)
			cout<<"\tPathState[LEFT]  = AROUND_CIRCLE"<<endl;
		else
			cout<<"\tPathState[LEFT]  = undefined!"<<endl;
	}
	else if(ArmType == RIGHT_ARM)
	{
		if(PathState == MOVETO_CIRCLE)
  			cout<<"\tPathState[RIGHT] = MOVETO_CIRCLE"<<endl;

		else if(PathState == AROUND_CIRCLE)
			cout<<"\tPathState[RIGHT]  = AROUND_CIRCLE"<<endl;
		else
			cout<<"\tPathState[RIGHT]  = undefined!"<<endl;
	}
	
}



/**
*	\fn void show_Distance()
*
* 	\brief displays current distance to center position
*
* 	\param void
*
*	\return void
*/
void Raven_PathPlanner::show_Distance()
{
	if(ArmType == LEFT_ARM)
	{
		cout<<"\tDistance from Center[LEFT]  = "<<Distance/10000<<" cm "<<endl;
	}
	else if(ArmType == RIGHT_ARM)
	{
		cout<<"\tDistance from Center[RIGHT]  = "<<Distance/10000<<" cm "<<endl;
	}
	
}


/**
*	\fn void show_Center()
*
* 	\brief displays Center point value to console
*
* 	\param void
*
*	\return void
*/
void Raven_PathPlanner::show_Center()
{
	tfScalar x = Center.getX();
	tfScalar y = Center.getY();
	tfScalar z = Center.getZ();

	if(ArmType == LEFT_ARM)
	{
		cout<<"\tCenter[LEFT] = ("<<x<<","<<y<<","<<z<<")"<<endl;

	}
	else if(ArmType == RIGHT_ARM)
	{
		cout<<"\tCenter[RIGHT] = ("<<x<<","<<y<<","<<z<<")"<<endl;
	}
}



/**
*	\fn void show_delPos()
*
* 	\brief displays Delta_Pos values to console
*
* 	\param void
*
*	\return void
*/
void Raven_PathPlanner::show_delPos()
{
	tfScalar x = Delta_Pos.getX();
	tfScalar y = Delta_Pos.getY();
	tfScalar z = Delta_Pos.getZ();

	if(ArmType == LEFT_ARM)
	{
		cout<<"\tdelPos[LEFT] = ("<<x<<","<<y<<","<<z<<")"<<endl;

	}
	else if(ArmType == RIGHT_ARM)
	{
		cout<<"\tdelPos[RIGHT] = ("<<x<<","<<y<<","<<z<<")"<<endl;
	}
	
	
}



/**
*	\fn tfScalar DistanceOf(tf::Vector3 point1,tf::Vector3 point2)
*
* 	\brief this function computes the distance between two tf::Vector3 points
*
* 	\param tf::Vector3, tf::Vector3
*
*	\return tfScalar
*/
tfScalar Raven_PathPlanner::DistanceOf(tf::Vector3 point1,tf::Vector3 point2)
{
	tfScalar DX = point1.getX() - point2.getX();
	tfScalar DY = point1.getY() - point2.getY();
	tfScalar DZ = point1.getZ() - point2.getZ();

	return sqrt(DX*DX+DY*DY+DZ*DZ);
}



/**
*	\fn tf::Transform ComputeCircleTrajectory()
*
* 	\brief this function delta motion commands for RAVEN to follow circular trajectory
*
* 	\param void
*
*	\return tf::Transform
*/
tf::Transform Raven_PathPlanner::ComputeCircleTrajectory()
{
	tf::Transform TF_INCR;

	pthread_mutex_lock(&data1Mutex);
	// (1) set position increment (only Y-Z plane)
	switch(PathState)
	{
		case MOVETO_CIRCLE: // finding orbit case

			if(Distance == 0) // exactly at center
			{
				if(Base_Plane == XZ_PLANE)
					Delta_Pos.setValue(Speed,0,0); 	
				else
					Delta_Pos.setValue(0,Speed,0); 	
			}
			else // either inside of outside circle
			{
				tf::Vector3 Delta_Pos1 = TuneRadiusMotion();  // normal direction
				tf::Vector3 Delta_Pos2 = AutoCircleMotion4(); // tangent direction
				
				//..
				K = Kp * tfPow(Error,Modi_Dista_Pow);

				K = (K>1) ? 1 : K; // K should not be larger than 1

				Delta_Pos = (K)*Delta_Pos1 + (1-K)*Delta_Pos2;

				Delta_Pos = Delta_Pos.normalized()*Speed;  //..
			}
			break;

		case AROUND_CIRCLE:  // in orbit case

			Delta_Pos = AutoCircleMotion4();
			break;

		default:
			ROS_ERROR("Undefined PathState. We can only send NULL Trajectory!");
			Delta_Pos.setValue(0,0,0);
			break;	
	}

	// (2) no rotation increment
	tfScalar  W = 1;
	tfScalar QX = 0;
	tfScalar QY = 0;
	tfScalar QZ = 0;			
	tf::Quaternion Delta_Ori(QX,QY,QZ,W);

	// (3) add increment to return variable
	TF_INCR.setOrigin(Delta_Pos);   
	TF_INCR.setRotation(Delta_Ori); 
	pthread_mutex_unlock(&data1Mutex);

	return TF_INCR;
}



/**
*	\fn tf::Transform ComputeNullTrajectory()
*
* 	\brief this function generates commands to let RAVEN stay stationary
*
* 	\param void
*
*	\return tf::Transform
*/
tf::Transform Raven_PathPlanner::ComputeNullTrajectory()
{
	tf::Transform TF_INCR;
	
	tfScalar X = 0;
	tfScalar Y = 0;
	tfScalar Z = 0;
	Delta_Pos.setValue(X,Y,Z); 

	tfScalar  W = 1;
	tfScalar QX = 0;
	tfScalar QY = 0;
	tfScalar QZ = 0;			
	tf::Quaternion q_temp(QX,QY,QZ,W);

	TF_INCR.setOrigin(Delta_Pos);   //add position increment
	TF_INCR.setRotation(q_temp);    //add rotation increment

	return TF_INCR;
}



/**
*	\fn void AutoCircleMotion1()
*
* 	\brief this is the first algorithm for circle trajectory generation
*	       (not very stable. eventually unused!)
*
* 	\param void
*
*	\return void
*/
void Raven_PathPlanner::AutoCircleMotion1()
{
	Delta_Pos = Current_Pos - Center;
	Delta_Pos.setX(0);
	
	if(Distance > Radius)
	{
		tfScalar a = (Distance*Distance-Radius*Radius+Speed*Speed)/(2*Distance);
		tfScalar h = sqrt(Speed*Speed - a*a);

		tf::Vector3 P0 = Center;
		tf::Vector3 P1 = Current_Pos;
		tf::Vector3 P2 = P0 + Delta_Pos.normalized()*a;
		tf::Vector3 P3;

		tfScalar Y3 = P2.getY()+h*(P1.getZ()-P0.getZ())/Distance;
		tfScalar Z3 = P2.getZ()-h*(P1.getY()-P0.getY())/Distance;
		P3.setValue(0,Y3,Z3);

		tf::Vector3 tmp(0,P3.getY()-P0.getY(),P3.getZ()-P0.getZ()); //vector P0P3
		tf::Vector3 Cross_product = Delta_Pos.cross(tmp);

		if(Cross_product.getX() * Direction > 0)
		{	
			// correct direction
			Delta_Pos = tmp - Delta_Pos;
		}
		else
		{
			// need to flip direction
			tfScalar Y3 = P2.getY()-h*(P1.getZ()-P0.getZ())/Distance; 
			tfScalar Z3 = P2.getZ()+h*(P1.getY()-P0.getY())/Distance;
			P3.setValue(0,Y3,Z3); 

			tmp.setValue(0,P3.getY()-P0.getY(),P3.getZ()-P0.getZ()); //vector P0P3
			Delta_Pos = tmp - Delta_Pos;
			
		}

	}
	else if (Distance < Radius)
	{
		tfScalar a = (-Distance*Distance+Radius*Radius+Speed*Speed)/(2*Distance);
		tfScalar h = sqrt(Speed*Speed - a*a);

		tf::Vector3 P0 = Center;
		tf::Vector3 P1 = Current_Pos;
		tf::Vector3 P2 = P0 + Delta_Pos.normalized()*(Distance + a);
		tf::Vector3 P3;
		
		tfScalar Y3 = P2.getY()+h*(P1.getZ()-P0.getZ())/Distance;
		tfScalar Z3 = P2.getZ()-h*(P1.getY()-P0.getY())/Distance;
		P3.setValue(0,Y3,Z3);

		tf::Vector3 tmp(0,P3.getY()-P0.getY(),P3.getZ()-P0.getZ()); //vector P0P3
		tf::Vector3 Cross_product = Delta_Pos.cross(tmp);

		if(Cross_product.getX() * Direction > 0)
		{	
			// correct direction
			Delta_Pos = tmp - Delta_Pos;
		}
		else
		{
			// need to flip direction
			tfScalar Y3 = P2.getY()-h*(P1.getZ()-P0.getZ())/Distance; 
			tfScalar Z3 = P2.getZ()+h*(P1.getY()-P0.getY())/Distance;
			P3.setValue(0,Y3,Z3); 

			tmp.setValue(0,P3.getY()-P0.getY(),P3.getZ()-P0.getZ()); //vector P0P3
			Delta_Pos = tmp - Delta_Pos;
			
		}
		
	}
	else // Distance == Radius (move towards tangent direction)
	{		
		tfScalar Y = Delta_Pos.getZ();
		tfScalar Z = -Delta_Pos.getY();
		
		tf::Vector3 Tangent_Pos(0,Y,Z);
		tf::Vector3 Cross_product = Delta_Pos.cross(Tangent_Pos);

		// [Note:] Direction == 1 : counter-clockwise
		//	   Direction == -1: clockwise

		if(Cross_product.getX() * Direction > 0) // correct direction	
			 Delta_Pos = Tangent_Pos;
		else
			 Delta_Pos = -Tangent_Pos; // need to flip direction
			
		Delta_Pos.normalize();
		Delta_Pos *= Speed;
	}	
}



/**
*	\fn void AutoCircleMotion2()
*
* 	\brief this is the second algorithm for circle trajectory generation
*
* 	\param void
*
*	\return void
*/
void Raven_PathPlanner::AutoCircleMotion2()
{
	tfScalar    del_angle;
	tf::Vector3 now_Vector;
	tf::Vector3 nxt_Vector;
	tf::Vector3 del_Vector;
	tf::Vector3 X_AXIS(1,0,0);

	now_Vector = Current_Pos - Center;
	now_Vector.setX(0);

	del_angle = min( asin(Speed/Radius) , M_PI/6 );

	nxt_Vector = now_Vector.rotate(X_AXIS,del_angle*Direction);
	nxt_Vector = nxt_Vector.normalized()*Radius;

	del_Vector = nxt_Vector - now_Vector;

	if(del_Vector.length() > Speed)	
		del_Vector = del_Vector.normalized()*Speed;

	Delta_Pos.setValue(0,del_Vector.getY(),del_Vector.getZ());
}


/**
*	\fn void AutoCircleMotion3()
*
* 	\brief this is the third algorithm for circle trajectory generation
*              (the version that we actually use)
* 	\param void
*
*	\return void
*/
void Raven_PathPlanner::AutoCircleMotion3()
{
	tf::Vector3 now_Vector = Current_Pos - Center;
	tf::Vector3 del_Vector1,del_Vector2;

	tfScalar Y = now_Vector.getY();
	tfScalar Z = now_Vector.getZ();

	if(Y>0)
	{
		if(Z>0)
		{
			if(abs(Y)>=abs(Z)) // octant 1
			{
				del_Vector2.setValue(0,     0,Speed);
				del_Vector1.setValue(0,-Speed,Speed);
			}
			else // octant 2
			{
				del_Vector1.setValue(0,-Speed,    0);
				del_Vector2.setValue(0,-Speed,Speed);
			}
		}
		else if(Z<0)
		{
			if(abs(Y)>=abs(Z)) // octant 8
			{
				del_Vector1.setValue(0,    0,Speed);
				del_Vector2.setValue(0,Speed,Speed);
			}
			else // octant 7
			{
				del_Vector2.setValue(0,Speed,    0);
				del_Vector1.setValue(0,Speed,Speed);
			}
		}
		else // Z=0
		{
			del_Vector1.setValue(0,0,Speed);
			del_Vector2.setValue(0,0,Speed);
		}	
	}
	else if(Y<0)
	{
		if(Z>0)
		{
			if(abs(Y)>=abs(Z)) // octant 4
			{
				del_Vector1.setValue(0,             0,        -Speed);
				del_Vector2.setValue(0,-Speed/sqrt(2),-Speed/sqrt(2));
			}
			else // octant 3
			{
				del_Vector2.setValue(0,        -Speed,             0);
				del_Vector1.setValue(0,-Speed/sqrt(2),-Speed/sqrt(2));
			}
		}
		else if(Z<0)
		{
			if(abs(Y)>=abs(Z)) // octant 5
			{
				del_Vector2.setValue(0,            0,        -Speed);
				del_Vector1.setValue(0,Speed/sqrt(2),-Speed/sqrt(2));
			}
			else // octant 6
			{
				del_Vector1.setValue(0,        Speed,             0);
				del_Vector2.setValue(0,Speed/sqrt(2),-Speed/sqrt(2));
			}
		}
		else // Z=0
		{
			del_Vector1.setValue(0,0,-Speed);
			del_Vector2.setValue(0,0,-Speed);
		}	
	}
	else // Y=0
	{
		if(Z>0)
		{
			del_Vector1.setValue(0,-Speed,0);
			del_Vector2.setValue(0,-Speed,0);
		}
		else if(Z<0)
		{
			del_Vector1.setValue(0,Speed,0);
			del_Vector2.setValue(0,Speed,0);
		}
	}
	
	del_Vector1 = del_Vector1 * Direction;
	del_Vector2 = del_Vector2 * Direction;
	
	tf::Vector3 nxt_Vector = Current_Pos+(del_Vector1+del_Vector2)/2-Center;

	if(Direction == 1)
	{
		if(nxt_Vector.length() > Radius)
			Delta_Pos.setValue(0,del_Vector1.getY(),del_Vector1.getZ());
		else
			Delta_Pos.setValue(0,del_Vector2.getY(),del_Vector2.getZ());
	}
	else // Direction == -1
	{
		if(nxt_Vector.length() > Radius)
			Delta_Pos.setValue(0,del_Vector2.getY(),del_Vector2.getZ());
		else
			Delta_Pos.setValue(0,del_Vector1.getY(),del_Vector1.getZ());
	}

}



/**
*	\fn tf::Vector3 AutoCircleMotion4()
*
* 	\brief this is the fourth algorithm for circle trajectory generation
*
* 	\param void
*
*	\return tf::Vector3 
*/
tf::Vector3 Raven_PathPlanner::AutoCircleMotion4()
{
	tfScalar del_angle;
	tf::Vector3 now_Vector,nxt_Vector,del_Vector;

	now_Vector = Current_Pos - Center;

	switch(Base_Plane)
	{
		case YZ_PLANE:
			now_Vector.setX(0);
		break;
		case XZ_PLANE:
			now_Vector.setY(0);
		break;
		case XY_PLANE:
			now_Vector.setZ(0);
		break;
	}

	now_Vector = now_Vector.normalized()*Radius;

	del_angle = min(2*asin(Speed/(2*Radius)) , M_PI/8 );

	switch(Base_Plane)
	{
		case YZ_PLANE:
			nxt_Vector = now_Vector.rotate(X_AXIS,del_angle*Direction);
		break;
		case XZ_PLANE:
			nxt_Vector = now_Vector.rotate(Y_AXIS,del_angle*Direction);
		break;
		case XY_PLANE:
			nxt_Vector = now_Vector.rotate(Z_AXIS,del_angle*Direction);
		break;
	}

	del_Vector = nxt_Vector - now_Vector;

	if(del_Vector.length() > Speed)	
		del_Vector = del_Vector.normalized()*Speed;

	switch(Base_Plane)
	{
		case YZ_PLANE:
			del_Vector.setX(0);
		break;
		case XZ_PLANE:
			del_Vector.setY(0);
		break;
		case XY_PLANE:
			del_Vector.setZ(0);
		break;
	}

	return  del_Vector;
}



/**
*	\fn tf::Vector3 TuneRadiusMotion()
*
* 	\brief this is the algorithm to help robot navigate to the correct radius value
*
* 	\param void
*
*	\return tf::Vector3 
*/
tf::Vector3 Raven_PathPlanner::TuneRadiusMotion()
{
	tf::Vector3 now_Vector,del_Vector;

	now_Vector = Current_Pos - Center;
	
	switch(Base_Plane)
	{
		case YZ_PLANE:
			now_Vector.setX(0);
		break;
		case XZ_PLANE:
			now_Vector.setY(0);
		break;
		case XY_PLANE:
			now_Vector.setZ(0);
		break;
	}

	if(Distance > Radius + STATE_THRESHOLD) // out of circle : need to move back in
	{
		del_Vector = now_Vector.normalized();

		if(Distance - Radius < Speed)
			del_Vector = - (Distance - Radius) * del_Vector;
		else
			del_Vector = - Speed * del_Vector;
		
	}
	else if(Distance < abs(Radius-STATE_THRESHOLD)) // inside circle : need to move outward
	{
		del_Vector = now_Vector.normalized();

		if(Radius - Distance < Speed)
			del_Vector = (Radius - Distance) * del_Vector;
		else
			del_Vector = Speed * del_Vector;
	}

	return del_Vector;
}

/*
// [DANGER]: for modification parameter tuning only

bool Raven_PathPlanner::set_Modi_Scale(int x)
{
	tfScalar step = 0.000001;
	Modi_Scale = Modi_Scale + step*x;
	return true;
}

bool Raven_PathPlanner::set_Modi_Speed_Pow(int x)
{
	tfScalar step = 0.1;
	Modi_Speed_Pow = Modi_Speed_Pow + step*x;
	return true;
}

bool Raven_PathPlanner::set_Modi_Dista_Pow(int x)
{
	tfScalar step = 0.5;
	Modi_Dista_Pow = Modi_Dista_Pow + step*x;
	return true;
}

tfScalar Raven_PathPlanner::get_Modi_Scale()
{
	tfScalar result = Modi_Scale;
	return result;
}

tfScalar Raven_PathPlanner::get_Modi_Speed_Pow()
{
	tfScalar result = Modi_Speed_Pow;
	return result;
}

tfScalar Raven_PathPlanner::get_Modi_Dista_Pow()
{
	tfScalar result = Modi_Dista_Pow;
	return result;
}
*/



