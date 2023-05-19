#include    "LinearMPC_header.h"

void LinearMPC_app::getReference(
    Eigen::MatrixXd state
){
    int Nx = parameters.Nx;

    int Nu = parameters.Nu;

    int Np = parameters.Np;

    double stepTime = parameters.stepTime;

    Eigen::MatrixXd t_vector{Np,1};

    for (size_t i = 0; i < Np; i++)
    {
        t_vector(i,0) = (i + 1)*stepTime;
    }

    double v_ref = parameters.v_ref;

    Eigen::MatrixXd x_ref_vector{Np,1};

    Eigen::MatrixXd y_ref_vector{Np,1};

    Eigen::MatrixXd phi_ref_vector{Np,1};

    int mode = 2;

    if (mode == 1)//直线轨迹
    {
        x_ref_vector = state(0,0) + (v_ref*t_vector).array();

        y_ref_vector = 1 + (0*t_vector).array();

        phi_ref_vector = 0*t_vector;
    }
    else if (mode == 2)//圆轨迹
    {
        double R_ref = 2;

        double omega_ref = v_ref/R_ref;

        double center_x = 0;

        double center_y = 3;

        phi_ref_vector = omega_ref*t_vector;

        double theta = atan2(state(1,0) - center_y , state(0,0) - center_x);

        x_ref_vector = center_x + R_ref*cos(theta + phi_ref_vector.array());

        y_ref_vector = center_y + R_ref*sin(theta + phi_ref_vector.array());

        phi_ref_vector = phi_ref_vector.array() + theta + PI/2;
    }
    else if (mode == 3)//泊车
    {
        double x_pose = 5;

        double y_pose = 3;

        double phi_pose = -PI;

        x_ref_vector = Eigen::MatrixXd::Constant(Np,1,x_pose);

        y_ref_vector = Eigen::MatrixXd::Constant(Np,1,y_pose);

        phi_ref_vector = Eigen::MatrixXd::Constant(Np,1,phi_pose);
    }
    else if (mode == 4)//跟踪通过话题发布的全局路径，需要进行路径平滑
    {
        printf("暂时还没做");
    }
    else
    {
        printf("invalid mode");
    }
  
    observe_reference = Eigen::MatrixXd::Constant(Nx + Nu,Np,0);

    observe_reference.block(0,0,1,Np) = x_ref_vector.transpose();

    observe_reference.block(1,0,1,Np) = y_ref_vector.transpose();

//    for (size_t i = 0; i < phi_ref_vector.size(); i++)
//    {
//        phi_ref_vector(i) = mod2pi(phi_ref_vector(i));
//    }
    
    observe_reference.block(2,0,1,Np) = phi_ref_vector.transpose();

    observe_reference.block(3 , 0 , 1 , Np) = Eigen::MatrixXd::Zero(1 , Np);

    observe_reference.block(4 , 0 , 1 , Np) = Eigen::MatrixXd::Constant(1 , Np , parameters.v_ref);

    solution_reference = Eigen::MatrixXd::Zero(Np*Nu , 1);

    // std::cout<<x_ref_vector<<std::endl;
    // std::cout<<y_ref_vector<<std::endl;
    // std::cout<<phi_ref_vector<<std::endl;
}
