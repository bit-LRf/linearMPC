#include    "LinearMPC_header.h"

LinearMPC_app::LinearMPC_app(ros::NodeHandle& nh)
{
    position_update_flag = false;

    state_subscriber = nh.subscribe("/odom",1,&LinearMPC_app::subOdomTopicCallBack,this);

    path_subscriber = nh.subscribe("/planning/path",1,&LinearMPC_app::subPathTopicCallBack,this);

    cmd_publisher = nh.advertise<geometry_msgs::Twist>("/mpc/cmd_vel",5,true);

    opt_path_publisher = nh.advertise<nav_msgs::Path>("/mpc/opt_path",5,true);

    getParameters(parameters);

    mpcInitialize();
}

LinearMPC_app::~LinearMPC_app()
{
}

void LinearMPC_app::mpcInitialize(){
    int Np = parameters.Np;
    int Nx = parameters.Nx;
    int Nu = parameters.Nu;

    control_n1 = parameters.control_start;

    solver = new OsqpEigen::Solver;

    accumulation_matrix = Eigen::MatrixXd::Constant(Np,Np,0);

    for (int i = 0; i < Np; i++)
    {
        for (int j = 0; j < Np; j++)
        {
            if (i >= j)
            {
                accumulation_matrix(i,j) = 1;
            }
        }
    }

    accumulation_matrix = Eigen::kroneckerProduct(accumulation_matrix,Eigen::MatrixXd::Identity(Nu,Nu)).eval();

    state = Eigen::MatrixXd::Zero(Nx,1);
    state_0 = state;
    state_n1 = state;
    state_eq = state;
}

bool LinearMPC_app::mpcUpdates(){
    int Np = parameters.Np;
    int Nx = parameters.Nx;
    int Nu = parameters.Nu;

    ros::spinOnce();

    if(!position_update_flag)
    {
        return false;
    }
    position_update_flag = false;

    solution = Eigen::MatrixXd::Zero(Np*Nu,1);

    state_0 = getNextState(state_n1 , control_n1);

    double x_0 = state_0(0);
    double y_0 = state_0(1);
    double phi_0 = state_0(2);

    if(!getReference(state_0))
    {
        return false;
    }

    transition_matrix <<
            1 , 0 , 0 , -x_0,
            0 , 1 , 0 , -y_0,
            0 , 0 , 1 , 0 ,
            0 , 0 , 0 , 1;
    // std::cout<<transition_matrix<<std::endl;

    rotation_matrix <<
            cos(phi_0) , sin(phi_0) , 0 , 0 ,
            -sin(phi_0) , cos(phi_0) , 0 , 0 ,
            0 , 0 , 1 , 0 ,
            0 , 0 , 0 , 1;
    // std::cout<<rotation_matrix<<std::endl;

    Eigen::MatrixXd observe_relative_reference = Eigen::MatrixXd::Zero(4 , Np);

    observe_relative_reference.block(0 , 0 , 2 , Np) = observe_reference.block(0 , 0 , 2 , Np);
    observe_relative_reference.block(3 , 0 , 1 , Np) = Eigen::MatrixXd::Constant(1 , Np , 1);

    observe_relative_reference = rotation_matrix*transition_matrix*observe_relative_reference;

    observe_reference.block(0 , 0 , 2 , Np) = observe_relative_reference.block(0 , 0 , 2 , Np);
    observe_reference.block(2 , 0 , 1 , Np) = observe_reference.block(2 , 0 , 1 , Np).array() - phi_0;

    for (int i = 0; i < Np; i++)
    {
        observe_reference(2,i) = mod2pi(observe_reference(2,i));
    }

    observe_reference.resize(Np*(Nx + Nu),1);

//    mpcOptmizer_single_shooting(Eigen::MatrixXd::Zero(Nx , 1) , control_n1 , solution , observe_reference);
    mpcOptmizer_multiple_shooting(Eigen::MatrixXd::Zero(Nx , 1) , control_n1 , solution , observe_reference);

    control_output = control_n1 + solution.block(0 , 0 , 2 , 1);

    return true;
}

void LinearMPC_app::mpcOutputs(){
    int Np = parameters.Np;
    int Nx = parameters.Nx;
    int Nu = parameters.Nu;

    try
    {
        control_msg.angular.z = control_output(0);

        control_msg.linear.x = control_output(1);

        control_n1 = control_output;

        cmd_publisher.publish(control_msg);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }

    try
    {
        nav_msgs::Path opt_path_msg;

        observe_optmize.resize(Nx + Nu , Np);
        Eigen::MatrixXd observe_relative_optmize = Eigen::MatrixXd::Zero(4 , Np);

        observe_relative_optmize.block(0 , 0 , 2 , Np) = observe_optmize.block(0 , 0 , 2 , Np);
        observe_relative_optmize.block(3 , 0 , 1 , Np) = Eigen::MatrixXd::Constant(1 , Np , 1);

        observe_relative_optmize = (rotation_matrix*transition_matrix).inverse()*observe_relative_optmize;

        observe_optmize.block(0 , 0 , 2 , Np) = observe_relative_optmize.block(0 , 0 , 2 , Np);

        opt_path_msg.header.frame_id = "/map";

        geometry_msgs::PoseStamped pose;

        for (int i = 0; i < Np; i++)
        {
            pose.pose.position.x = observe_optmize(0,i);
            pose.pose.position.y = observe_optmize(1,i);
            opt_path_msg.poses.insert(opt_path_msg.poses.end() , pose);
        }
        
        opt_path_publisher.publish(opt_path_msg);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}
