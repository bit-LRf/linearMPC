#include    "LinearMPC_header.h"

void LinearMPC_app::getDiscreteMatrix(
    Eigen::MatrixXd state_k,
    Eigen::MatrixXd control_k
){
    double phi_k = state_k(2,0);

    // std::cout<<control_k<<std::endl;

    double  omega_k = control_k(0,0);

    double v_k = control_k(1,0);

    double stepTime = parameters.stepTime;

    Eigen::MatrixXd X_dot_k{3,1};

    X_dot_k << v_k*cos(phi_k) , v_k*sin(phi_k) , omega_k;

    Eigen::MatrixXd DfDX_k{3,3};

    DfDX_k << 
    0 , 0 , -v_k*sin(phi_k) , 
    0 , 0 , v_k*cos(phi_k) , 
    0 , 0 , 0;

    Eigen::MatrixXd DfDu_k{3,2};

    DfDu_k << 
    0 , cos(phi_k) , 
    0 , sin(phi_k) , 
    1 , 0;

    state_matrix_A_continuous = DfDX_k;
    state_matrix_B_continuous = DfDu_k;
    state_matrix_C_continuous = X_dot_k  - DfDX_k*state_k - DfDu_k*control_k;

    state_matrix_A_discrete = Eigen::MatrixXd::Identity(parameters.Nx,parameters.Nx) + stepTime*state_matrix_A_continuous;
    state_matrix_B_discrete = stepTime*state_matrix_B_continuous;
    state_matrix_C_discrete = stepTime*state_matrix_C_continuous;

    // std::cout<<"\n离散系统矩阵A:"<<std::endl;
    // std::cout<<state_matrix_A_discrete<<std::endl;
    // std::cout<<"\n离散系统矩阵B:"<<std::endl;
    // std::cout<<state_matrix_B_discrete<<std::endl;
    // std::cout<<"\n离散系统矩阵C:"<<std::endl;
    // std::cout<<state_matrix_C_discrete<<std::endl;
}
