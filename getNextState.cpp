#include    "LinearMPC_header.h"

Eigen::MatrixXd LinearMPC_app::getNextState(
    Eigen::MatrixXd state,
    Eigen::MatrixXd control
){
    Eigen::MatrixXd next_state{3,1};

    double x_k = state(0,0);
    
    double  y_k = state(1,0);

    double phi_k = state(2,0);

    double  omega_k = control(0,0);

    double v_k = control(1,0);

    double stepTime = parameters.stepTime;

    if (omega_k == 0)
    {
        next_state(0,0) = x_k + stepTime*v_k*cos(phi_k);
        next_state(1,0) = y_k + stepTime*v_k*sin(phi_k);
        next_state(2,0) = phi_k;
    }
    else
    {
        double theta = stepTime*omega_k;
        double R = v_k/omega_k;
        next_state(0,0) = x_k + R*(sin(phi_k + theta) - sin(phi_k));
        next_state(1,0) = y_k + R*(cos(phi_k) - cos(phi_k + theta));
        next_state(2,0) = phi_k + theta;
    }

//    next_state(2) = mod2pi(next_state(2));
    
    return next_state;
}
