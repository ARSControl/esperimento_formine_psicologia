#include "URKDL/URKDL.h"

int main(int argc, char** argv)
{
	
	ros::init(argc, argv, "ur_kdl");
	URKDL ce;
	ros::Rate r(10);

	while(ros::ok())
	{
		
		ce.spinner();
		r.sleep();

	}

	return 0;

}
