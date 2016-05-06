#include <vo_flow/imu_flow_fusion.h>

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"ImuFlowFusionNode");
    ImuFlowFusion imuflowfusion;
    ros::spin();
    return 0;
}
