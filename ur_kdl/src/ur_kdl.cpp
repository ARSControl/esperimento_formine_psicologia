#include "URKDL/URKDL.h"

URKDL::URKDL()
{  
	service = nh.advertiseService("inverse_kinematics", &URKDL::ik, this);
	service_lma = nh.advertiseService("inverse_kinematics_lma", &URKDL::ik_lma, this);

	//geometria parametri DH(a, alpha, d, theta)
	ur10e.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0,M_PI_2,0.1807,0.0)));
	ur10e.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(-0.6127,0.0,0.0,0.0)));
	ur10e.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(-0.57155,0.0,0.0,0.0)));
	ur10e.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0,M_PI_2,0.17415,0.0)));
	ur10e.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0,-M_PI_2,0.11985,0.0)));
	ur10e.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame::DH(0.0,0.0,0.11655,0.0)));

	fksolver = new KDL::ChainFkSolverPos_recursive(ur10e);
	iksolver_v = new KDL::ChainIkSolverVel_pinv(ur10e);
	iksolver = new KDL::ChainIkSolverPos_NR(ur10e,*fksolver,*iksolver_v,100,1e-6);//Maximum 100 iterations, stop at accuracy 1e-6
	iksolver_lma = new KDL::ChainIkSolverPos_LMA(ur10e, 1e-5,500,1e-15);
	jsolver = new KDL::ChainJntToJacSolver(ur10e);

	q.resize((ur10e.getNrOfJoints()));
	q_init.resize((ur10e.getNrOfJoints()));

	for(int i = 0; i < ur10e.getNrOfJoints(); i++)
	{
		q(i) = 0.0;
		q_init(i) = 0.0;
	}

	lock_joints.resize(ur10e.getNrOfJoints());
	for(int i = 0; i < ur10e.getNrOfJoints(); i++){
		lock_joints[i] = false;
	}
	j.resize(6);

	jsolver->setLockedJoints(lock_joints);
	jsolver->JntToJac(q, j, ur10e.getNrOfJoints());
	for(int i = 0; i < ur10e.getNrOfJoints(); i++)
	{
		for(int l = 0; l < ur10e.getNrOfJoints(); l++)
		{
			std::cout << j(i,l) << " ";
		}
		std::cout << std::endl;
	}
}

bool URKDL::ik(ur_kdl::IK::Request  &req, ur_kdl::IK::Response &res)
{
	KDL::Rotation o = KDL::Rotation::Quaternion(req.ee_pose.orientation.x, req.ee_pose.orientation.y, req.ee_pose.orientation.z, req.ee_pose.orientation.w);
	KDL::Vector p(req.ee_pose.position.x, req.ee_pose.position.y, req.ee_pose.position.z);

	ee_pose = KDL::Frame(o, p);

	for(int i = 0; i < ur10e.getNrOfJoints(); i++)
	{
		q_init(i) = req.actual_joint_state.position[i];
	}

	iksolver->CartToJnt(q_init, ee_pose, q);

	res.joint_values.position.resize(ur10e.getNrOfJoints());

	for(int i = 0; i < ur10e.getNrOfJoints(); i++)
	{	
		res.joint_values.position[i] = q(i);
	}

	return true;
}

bool URKDL::ik_lma(ur_kdl::IK::Request  &req, ur_kdl::IK::Response &res)
{
	KDL::Rotation o = KDL::Rotation::Quaternion(req.ee_pose.orientation.x, req.ee_pose.orientation.y, req.ee_pose.orientation.z, req.ee_pose.orientation.w);
	KDL::Vector p(req.ee_pose.position.x, req.ee_pose.position.y, req.ee_pose.position.z);

	ee_pose = KDL::Frame(o, p);

	for(int i = 0; i < ur10e.getNrOfJoints(); i++)
	{
		q_init(i) = req.actual_joint_state.position[i];
	}

	iksolver_lma->CartToJnt(q_init, ee_pose, q);

	res.joint_values.position.resize(ur10e.getNrOfJoints());

	for(int i = 0; i < ur10e.getNrOfJoints(); i++)
	{	
		res.joint_values.position[i] = q(i);
	}

	return true;
}

void URKDL::spinner()
{
	ros::spinOnce();
}
