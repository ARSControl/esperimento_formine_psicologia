#ifndef DRIVER_H
#define DRIVER_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Float64MultiArray.h"
#include "math.h"
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "ur_kdl/IK.h"
// #include "gripper_control/GripperControl.h"

#include "ur_rtde_controller/RobotiQGripperControl.h"

#include<iostream>
//#include<conio.h>

#include <signal.h>
#include <termios.h>
#include <stdio.h>


class Posdriver
{
	public:
		Posdriver();
		void spinner(void);
	private:
		ros::NodeHandle nh;
		void DataCallback(const sensor_msgs::JointState::ConstPtr& msg);
		void ArduinoCallback(const std_msgs::Int8::ConstPtr& piece);
		ros::Publisher PosJoints;
		ros::Publisher PosPlace;
        // trajectory_msgs::JointTrajectory posdr;
        trajectory_msgs::JointTrajectoryPoint posdr;
		std_msgs::Int8 pos_pla;
		ros::Subscriber PosJoints_actual;
		ros::Subscriber PosPiece;
		int numbp=0;
		int j=0;
		int phase=0;
		int k=0;
		int newmess = 0;
		//char key_press;
		std::vector<float> pos;
		std::vector<float> reachpos;
		std::vector<float> actualpos;
		std::vector<float> diffpos;
		std::vector<float> Px;
		std::vector<float> Py;
		std::vector<float> PxT;
		std::vector<float> PyT;
		float Px1;
		float Py1;
		float Px2;
		float Py2;
		float PosHomex;
		float PosHomey;
		float PosHomez;
		float PosTappx;
		float PosTappy;
		float PosTappz;
		float Posz;
		float alfa;
		float beta;
		float Pi_G = 3.14159265358979323846;
		float quatx = 0;
		float quaty = 0;
		float quatz = 0;
		float quatw = 0;
		float actpositionx = 0;
		float actpositiony = 0;
		float actpositionz = 0;
		float despositionx = 0;
		float despositiony = 0;
		float despositionz = 0;
		float vel = 0;
		float timeto = 0;
		ros::ServiceClient inverse;
        ur_kdl::IK srv;
		// GripperControl gripper;
		ros::ServiceClient gripper_client;
		void moveGripper(float position, float speed, float force);
		void moveHome(float x, float y, float z, float ox, float oy, float oz, float ow);

};

#endif /* POSITION_DRIVER_H */