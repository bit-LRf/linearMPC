#include    "LinearMPC_header.h"

void LinearMPC_app::subOdomTopicCallBack(
    const nav_msgs::Odometry& msg
){
    std::cout<<"update position"<<std::endl;

    odom_msg = msg;

    state_n1(0) = odom_msg.pose.pose.position.x;
    state_n1(1) = odom_msg.pose.pose.position.y;

    tf::Quaternion qtn;
    tf::quaternionMsgToTF(odom_msg.pose.pose.orientation,qtn);
    double roll,pitch,yaw;
    tf::Matrix3x3(qtn).getRPY(roll,pitch,yaw);
    state_n1(2) = mod2pi(yaw);

    position_update_flag = true;
}

void LinearMPC_app::subPathTopicCallBack(
    const   nav_msgs::Path& msg
){
    std::cout<<"update reference path"<<std::endl;

    ref_path_msg = msg;
}
