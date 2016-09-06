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
*	\fn bool set_Center(boost::array<int, 6> center)
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

	if(Distance == 0 || Distance > Radius+Speed || Distance < abs(Radius-Speed))
		PathState = MOVETO_CIRCLE;
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
  		cout<<"PathState[LEFT] = MOVETO_CIRCLE\t";

		else if(PathState == AROUND_CIRCLE)
			cout<<"PathState[LEFT]  = AROUND_CIRCLE\t";
		else
			cout<<"PathState[LEFT]  = undefined!\t";
	}
	else if(ArmType == RIGHT_ARM)
	{
		if(PathState == MOVETO_CIRCLE)
  		cout<<"PathState[RIGHT] = MOVETO_CIRCLE\t";

		else if(PathState == AROUND_CIRCLE)
			cout<<"PathState[RIGHT]  = AROUND_CIRCLE\t";
		else
			cout<<"PathState[RIGHT]  = undefined!\t";
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
		cout<<"Center[LEFT]("<<x<<","<<y<<","<<z<<")\t";

	}
	else if(ArmType == RIGHT_ARM)
	{
		cout<<"Center[RIGHT]("<<x<<","<<y<<","<<z<<")\t";
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
		cout<<"delPos[LEFT]("<<x<<","<<y<<","<<z<<")\t";

	}
	else if(ArmType == RIGHT_ARM)
	{
		cout<<"delPos[RIGHT]("<<x<<","<<y<<","<<z<<")\t";
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

	// (1) set position increment (only Y-Z plane)
	switch(PathState)
	{
		case MOVETO_CIRCLE: // finding orbit case

			Delta_Pos = Current_Pos - Center;
			Delta_Pos.setX(0);  //only care about the YZ plane

			if(Distance == 0) // at center
			{
				Delta_Pos.setValue(0,Speed,0);
			}
			else if(Distance > Radius + Speed) // out of circle : need to move back in
			{
				Delta_Pos.normalize();

				if(Distance - Radius < Speed)
					Delta_Pos = - (Distance - Radius) * Delta_Pos;
				else
					Delta_Pos = - Speed * Delta_Pos;
			}
			else if(Distance < abs(Radius-Speed)) // inside circle : need to move outward
			{
				Delta_Pos.normalize();

				if(Radius - Distance < Speed)
					Delta_Pos = (Radius - Distance) * Delta_Pos;
				else
					Delta_Pos = Speed * Delta_Pos;
			}

			break;

		case AROUND_CIRCLE:  // in orbit case
			AutoCircleMotion2();
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
	tf::Vector3 tmpVec(X,Y,Z); 

	tfScalar  W = 1;
	tfScalar QX = 0;
	tfScalar QY = 0;
	tfScalar QZ = 0;			
	tf::Quaternion q_temp(QX,QY,QZ,W);

	TF_INCR.setOrigin(tmpVec);   //add position increment
	TF_INCR.setRotation(q_temp); 		  //add rotation increment

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
*              (the version that we actually use)
*
* 	\param void
*
*	\return void
*/
void Raven_PathPlanner::AutoCircleMotion2()
{
	tfScalar angle, del_angle;
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

	if(del_Vector.length() > DEL_POS_THRESHOLD)	
		del_Vector = del_Vector.normalized()*DEL_POS_THRESHOLD;

	Delta_Pos.setValue(0,del_Vector.getY(),del_Vector.getZ());
}




