#ifndef SPARSE_FLOW_H_
#define SPARSE_FLOW_H_

#include <vo_flow/base_flow.h>

class SparseFlow: public BaseFlow
{
public:
    SparseFlow();
    ~SparseFlow();
    virtual void imageCallback(const sensor_msgs::Image::ConstPtr msg);
    virtual void heightCallback(const sensor_msgs::Range::ConstPtr msg);
    int point_num;
    vector<Point3i> point_vec;
    int err_qLevel;
    vector<Point2f> pre_points,cur_points;
};

#endif
