#include    "LinearMPC_header.h"

void LinearMPC_app::subOdomTopicCallBack(
    const nav_msgs::Odometry& msg
){
    odom_msg = msg;

    state_n1(0) = odom_msg.pose.pose.position.x;
    state_n1(1) = odom_msg.pose.pose.position.y;

    tf::Quaternion qtn;
    tf::quaternionMsgToTF(odom_msg.pose.pose.orientation,qtn);
    double roll,pitch,yaw;
    tf::Matrix3x3(qtn).getRPY(roll,pitch,yaw);
    state_n1(2) = mod2pi(yaw);
}

void LinearMPC_app::subPathTopicCallBack(
    const   nav_msgs::Path& msg
){
    ref_path_msg = msg;
}
