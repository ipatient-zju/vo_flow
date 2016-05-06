#include <vo_flow/sparse_flow.h>

int main(int argc, char *argv[])
{
	ros::init(argc,argv,"SparseFlowNode");
	SparseFlow sparse_flow;
	ros::spin();
	return 0;
}
