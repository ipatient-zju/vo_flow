#ifndef TEMPLATE_MATCH_FLOW_H_
#define TEMPLATE_MATCH_FLOW_H_

#include <vo_flow/base_flow.h>

class TemplateMatchFlow: public BaseFlow
{
public:
    TemplateMatchFlow();
    ~TemplateMatchFlow();
    int patch_num;
    int divide;
    int match_method;
    vector<Point3i> point_vec;
    virtual void imageCallback(const sensor_msgs::Image::ConstPtr msg);
    virtual void heightCallback(const sensor_msgs::Range::ConstPtr msg);
    //friend bool comp(const Point3i &a,const Point3i &b);
};

#endif
