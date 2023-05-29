#ifndef         LinearMPC
#define        LinearMPC

// ros env
#include    <ros/ros.h>
#include    <nav_msgs/Path.h>
#include    "nav_msgs/Odometry.h"
#include    "tf/transform_datatypes.h"
#include    "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include    "geometry_msgs/PoseStamped.h"
#include    "geometry_msgs/TransformStamped.h"
#include    "geometry_msgs/PointStamped.h"
#include    "geometry_msgs/Twist.h"

// osqp env
#include    "OsqpEigen/OsqpEigen.h"

//eigen env
#include    "Eigen/Dense"
#include    "eigen3/unsupported/Eigen/KroneckerProduct"

//others
#include    <iostream>
#include    <cmath>
#include    <math.h>
#include    <vector>
#include    <queue>
#include    <ctime>

const double PI = 3.1415926535; 

struct param
{
    int Nx;
    int Nu;
    int Np;

    double stepTime;

    double v_ref;

    Eigen::MatrixXd control_start;

    Eigen::MatrixXd control_max;
    Eigen::MatrixXd control_min;

    Eigen::MatrixXd delta_control_max;
    Eigen::MatrixXd delta_control_min;

    Eigen::MatrixXd observer_matrix_H;

    Eigen::VectorXd observe_weight_vector;

    Eigen::VectorXd solution_weight_vector;
};

struct distance_node
{
    double distance;

    int sequence;

    bool operator<(const distance_node &other)const
    {
        if(distance < other.distance)
        {
            return false;
        }
        else {
            return true;
        }
    }
};


class LinearMPC_app
{
private:
    OsqpEigen::Solver* solver;

    Eigen::MatrixXd state_n1;

    Eigen::MatrixXd control_n1;

    Eigen::MatrixXd state_0;

    Eigen::MatrixXd state_eq;

    Eigen::MatrixXd control_eq;

    Eigen::MatrixXd state;

    Eigen::MatrixXd control;

    Eigen::MatrixXd solution;

    Eigen::MatrixXd observe_reference;

    Eigen::MatrixXd solution_reference;

    Eigen::MatrixXd observe_optmize;

    Eigen::MatrixXd accumulation_matrix;

    Eigen::MatrixXd state_matrix_A_continuous;

    Eigen::MatrixXd state_matrix_B_continuous;
    
    Eigen::MatrixXd state_matrix_C_continuous;

    Eigen::MatrixXd state_matrix_A_discrete;

    Eigen::MatrixXd state_matrix_B_discrete;

    Eigen::MatrixXd state_matrix_C_discrete;

    Eigen::MatrixXd table_matrix_A;

    Eigen::MatrixXd single_shooting_matrix_A;

    Eigen::MatrixXd single_shooting_matrix_B;

    Eigen::MatrixXd generalized_control_B;
    
    Eigen::MatrixXd generalized_control_C;

    Eigen::MatrixXd control_output;

public:
    LinearMPC_app(ros::NodeHandle& nh);

    ~LinearMPC_app();

    Eigen::MatrixXd getNextState(
        Eigen::MatrixXd state,
        Eigen::MatrixXd control
    );

    void getDiscreteMatrix(
        Eigen::MatrixXd state,
        Eigen::MatrixXd control
    );

    bool getReference(
        Eigen::MatrixXd state
    );

    double mod2pi(
        double input
    );

    void getParameters(
        param& param
    );
    
    bool mpcOptmizer_single_shooting(
        Eigen::MatrixXd state_0,
        Eigen::MatrixXd control_n1,
        Eigen::MatrixXd solution_eq,
        Eigen::MatrixXd observe_reference
    );

    bool mpcOptmizer_multiple_shooting(
        Eigen::MatrixXd state_0,
        Eigen::MatrixXd control_n1,
        Eigen::MatrixXd solution_eq,
        Eigen::MatrixXd observe_reference
    );


    void mpcInitialize(

    );

    bool mpcUpdates(

    );

    void mpcOutputs(

    );

public:
    param parameters;

    bool position_update_flag;

private:
    clock_t startTime,endTime;

    std::priority_queue<distance_node> distance_queue;

    Eigen::Matrix4d transition_matrix;

    Eigen::Matrix4d rotation_matrix;

    nav_msgs::Odometry odom_msg;

    nav_msgs::Path ref_path_msg;

    ros::NodeHandle nh;

    ros::Subscriber state_subscriber;
    void subOdomTopicCallBack(const nav_msgs::Odometry& msg);

    ros::Subscriber path_subscriber;
    void subPathTopicCallBack(const nav_msgs::Path& msg);

    geometry_msgs::Twist control_msg;

    ros::Publisher  cmd_publisher;

    ros::Publisher  opt_path_publisher;
};

#endif

