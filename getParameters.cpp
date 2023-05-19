#include    "LinearMPC_header.h"

void LinearMPC_app::getParameters(param& param){
    param.Np = 20;

    param.Nx = 3;

    param.Nu = 2;

    param.stepTime = 0.05;

    param.v_ref = 0.5;

    Eigen::MatrixXd control_start{param.Nu,1};
    control_start <<
    0.25 ,
    0.5 ;
    param.control_start = control_start;

    Eigen::MatrixXd control_max{param.Nu,1};
    control_max <<
    1.0 , 
    1.1*param.v_ref;
    param.control_max = control_max;

    Eigen::MatrixXd control_min{param.Nu,1};
    control_min <<
    -1.0 , 
    -1.1*param.v_ref;
    param.control_min = control_min;

    Eigen::MatrixXd delta_control_max{param.Nu,1};
    delta_control_max <<
    2.0*param.stepTime , 
    1.0*param.stepTime;
    param.delta_control_max = delta_control_max;

    Eigen::MatrixXd delta_control_min{param.Nu,1};
    delta_control_min <<
    -2.0*param.stepTime , 
    -1.0*param.stepTime;
    param.delta_control_min = delta_control_min;

    Eigen::MatrixXd observer_matrix_H{param.Nx + param.Nu , param.Nx + param.Nu};
    observer_matrix_H <<
    1 , 0 , 0 , 0 , 0 , 
    0 , 1 , 0 , 0 , 0 , 
    0 , 0 , 1 , 0 , 0 ,
    0 , 0 , 0 , 0 , 0 ,
    0 , 0 , 0 , 0 , 0 ;
    param.observer_matrix_H = observer_matrix_H;

    Eigen::VectorXd observe_weight_vector{param.Nx + param.Nu};
    observe_weight_vector << 1 , 1 , 0.1 , 0 , 0;
    param.observe_weight_vector = 10*
    observe_weight_vector;

    Eigen::VectorXd solution_weight_vector{param.Nu};
    solution_weight_vector << 1 , 1;
    param.solution_weight_vector = 1*
    solution_weight_vector;

    double waitTime = 0.5;
    std::cout<<"参数加载完成"<<std::endl;
    ros::Duration(waitTime).sleep();//等待
    std::cout<<"\n初始控制量:"<<std::endl;
    std::cout<<param.control_start<<std::endl;
    ros::Duration(waitTime).sleep();//等待
    std::cout<<"\n预测长度:"<<std::endl;
    std::cout<<param.Np<<std::endl;
    ros::Duration(waitTime).sleep();//等待
    std::cout<<"\n步长:"<<std::endl;
    std::cout<<param.stepTime<<std::endl;
    ros::Duration(waitTime).sleep();//等待
    std::cout<<"\n参考速度:"<<std::endl;
    std::cout<<param.v_ref<<std::endl;
    ros::Duration(waitTime).sleep();//等待
    std::cout<<"\n控制量上界:"<<std::endl;
    std::cout<<param.control_max<<std::endl;
    ros::Duration(waitTime).sleep();//等待
    std::cout<<"\n控制量下界:"<<std::endl;
    std::cout<<param.control_min<<std::endl;
    ros::Duration(waitTime).sleep();//等待
    std::cout<<"\n控制增量上界:"<<std::endl;
    std::cout<<param.delta_control_max<<std::endl;
    ros::Duration(waitTime).sleep();//等待
    std::cout<<"\n控制增量下界:"<<std::endl;
    std::cout<<param.delta_control_min<<std::endl;
    ros::Duration(waitTime).sleep();//等待
    std::cout<<"\n观测量权重:"<<std::endl;
    std::cout<<param.observe_weight_vector<<std::endl;
    ros::Duration(waitTime).sleep();//等待
    std::cout<<"\n控制增量权重:"<<std::endl;
    std::cout<<param.solution_weight_vector<<std::endl;
    std::cout<<"--------------------\n"<<std::endl;
    ros::Duration(waitTime).sleep();//等待
}
