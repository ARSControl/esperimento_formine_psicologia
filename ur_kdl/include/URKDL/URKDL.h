#ifndef URKDL_H
#define URKDL_H

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include "ur_kdl/IK.h"


class URKDL
{
	public:
		URKDL();
		void spinner(void);
	private:
		bool ik(ur_kdl::IK::Request  &req, ur_kdl::IK::Response &res);
		bool ik_lma(ur_kdl::IK::Request  &req, ur_kdl::IK::Response &res);
		ros::NodeHandle nh;
		ros::ServiceServer service;
		ros::ServiceServer service_lma;

		KDL::Chain ur10e;
		KDL::ChainFkSolverPos_recursive* fksolver;//Forward position solver
		KDL::ChainIkSolverVel_pinv* iksolver_v;//Inverse velocity solver
		KDL::ChainIkSolverPos_NR* iksolver;//Inverse position solver
		KDL::ChainIkSolverPos_LMA* iksolver_lma;//Inverse position solver
		KDL::ChainJntToJacSolver* jsolver;//Jacobian solver

		KDL::JntArray q; //Future position of the robot
		KDL::JntArray q_init; //Actual position of the robot
		KDL::Frame ee_pose; //Desired cartesian position
		KDL::Jacobian j; //Jacobian
		std::vector<bool> lock_joints; //Locked joints to compute jacobian
};

#endif /* URKDL_H */
