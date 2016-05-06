#include <vo_flow/template_match_flow.h>

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"TemplateMatchFlowNode");
    TemplateMatchFlow template_match_flow;
    ros::spin();
    return 0;
}
