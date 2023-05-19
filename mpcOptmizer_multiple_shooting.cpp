#include    "LinearMPC_header.h"

bool LinearMPC_app::mpcOptmizer_multiple_shooting(
        Eigen::MatrixXd state_0,
        Eigen::MatrixXd control_n1,
        Eigen::MatrixXd solution_eq,
        Eigen::MatrixXd observe_reference
        ){
    int Np = parameters.Np;

    int Nx = parameters.Nx;

    int  Nu = parameters.Nu;

    //建立增量形式的矩阵

    startTime = clock();

    Eigen::MatrixXd control_n1_vector =  Eigen::kroneckerProduct(Eigen::MatrixXd::Constant(Np,1,1),control_n1);

    Eigen::MatrixXd control_eq_vector = accumulation_matrix*solution_eq + control_n1_vector;

    Eigen::MatrixXd control_eq;

    Eigen::MatrixXd state_eq;

    Eigen::MatrixXd constraint_eq_matrix = Eigen::MatrixXd::Zero(Np*(Nx + Nu) , (Np + 1)*(Nx + Nu) + Np*Nu);

    Eigen::MatrixXd bound_eq_matrix = Eigen::MatrixXd::Zero(Np*(Nx + Nu) , 1);

    Eigen::MatrixXd constraint_ineq_matrix = Eigen::MatrixXd::Identity((Np + 1)*(Nx + Nu) + Np*Nu , (Np + 1)*(Nx  + Nu) + Np*Nu);

    Eigen::MatrixXd upper_bound_ineq_matrix = Eigen::MatrixXd::Zero((Np + 1)*(Nx + Nu) + Np*Nu , 1);
    
    Eigen::MatrixXd lower_bound_ineq_matrix = Eigen::MatrixXd::Zero((Np + 1)*(Nx + Nu) + Np*Nu , 1);

    Eigen::MatrixXd constraint_matrix;

    Eigen::MatrixXd upper_bound;

    Eigen::MatrixXd lower_bound;

    state_eq = state_0;

    for (size_t i = 0; i < Np; i++)
    {
        control_eq = control_eq_vector.block(Nu*i , 0 , Nu , 1);

        getDiscreteMatrix(state_eq , control_eq);

        state_eq = getNextState(state_eq , control_eq);

        constraint_eq_matrix.block(i*(Nx + Nu) , i*(Nx + Nu) , Nx , Nx) = -state_matrix_A_discrete;

        constraint_eq_matrix.block(i*(Nx + Nu) , i*(Nx + Nu) + Nx , Nx , Nu) = -state_matrix_B_discrete;

        constraint_eq_matrix.block(i*(Nx + Nu) + Nx , i*(Nx + Nu) + Nx , Nu , Nu) = -Eigen::MatrixXd::Identity(Nu , Nu);

        constraint_eq_matrix.block(i*(Nx + Nu) , (i + 1)*(Nx + Nu) , (Nx + Nu) , (Nx + Nu)) = Eigen::MatrixXd::Identity(Nx + Nu , Nx + Nu);

        constraint_eq_matrix.block(i*(Nx + Nu) , (Np + 1)*(Nx + Nu) + i*Nu , Nx  , Nu) = -state_matrix_B_discrete;

        constraint_eq_matrix.block(i*(Nx + Nu) + Nx , (Np + 1)*(Nx + Nu) + i*Nu , Nu  , Nu) = -Eigen::MatrixXd::Identity(Nu , Nu);

        bound_eq_matrix.block(i*(Nx + Nu) , 0 , Nx ,1) = state_matrix_C_discrete;
    }

    upper_bound_ineq_matrix.block(0 , 0 , Nx , 1) = state_0;

    upper_bound_ineq_matrix.block(Nx , 0 , Nu , 1) = control_n1;

    Eigen::MatrixXd state_incremental_upper_bound = Eigen::MatrixXd::Zero(Nx + Nu , 1);
    state_incremental_upper_bound <<
            INFINITY,
            INFINITY,
            INFINITY,
            parameters.control_max(0),
            parameters.control_max(1);

    upper_bound_ineq_matrix.block(Nx + Nu , 0 , Np*(Nx + Nu) , 1) =
            Eigen::kroneckerProduct(Eigen::MatrixXd::Constant(Np , 1 , 1) , state_incremental_upper_bound);

    upper_bound_ineq_matrix.block((Np + 1)*(Nx + Nu) , 0 , Np*Nu , 1) =
            Eigen::kroneckerProduct(Eigen::MatrixXd::Constant(Np , 1 , 1) , parameters.delta_control_max);

    lower_bound_ineq_matrix.block(0 , 0 , Nx , 1) = state_0;

    lower_bound_ineq_matrix.block(Nx , 0 , Nu , 1) = control_n1;

    Eigen::MatrixXd state_incremental_lower_bound = Eigen::MatrixXd::Zero(Nx + Nu , 1);
    state_incremental_lower_bound <<
            -INFINITY,
            -INFINITY,
            -INFINITY,
            -parameters.control_max(0),
            -parameters.control_max(1);

    lower_bound_ineq_matrix.block(Nx + Nu , 0 , Np*(Nx + Nu) , 1) =
            Eigen::kroneckerProduct(Eigen::MatrixXd::Constant(Np , 1 , 1) , state_incremental_lower_bound);

    lower_bound_ineq_matrix.block((Np + 1)*(Nx + Nu) , 0 , Np*Nu , 1) =
            Eigen::kroneckerProduct(Eigen::MatrixXd::Constant(Np , 1 , 1) , parameters.delta_control_min);

    constraint_matrix = Eigen::MatrixXd::Zero(constraint_eq_matrix.rows() +  constraint_ineq_matrix.rows() , constraint_eq_matrix.cols());
    constraint_matrix.block(0 , 0 , constraint_eq_matrix.rows() , constraint_eq_matrix.cols()) = constraint_eq_matrix;
    constraint_matrix.block(constraint_eq_matrix.rows() , 0 , constraint_ineq_matrix.rows() , constraint_ineq_matrix.cols()) = constraint_ineq_matrix;

    upper_bound = Eigen::MatrixXd::Zero(bound_eq_matrix.rows() + upper_bound_ineq_matrix.rows() , 1);
    upper_bound.block(0 , 0 , bound_eq_matrix.rows() , 1) = bound_eq_matrix;
    upper_bound.block(bound_eq_matrix.rows() , 0 , upper_bound_ineq_matrix.rows() , 1) = upper_bound_ineq_matrix;

    lower_bound = Eigen::MatrixXd::Zero(bound_eq_matrix.rows() + lower_bound_ineq_matrix.rows() , 1);
    lower_bound.block(0 , 0 , bound_eq_matrix.rows() , 1) = bound_eq_matrix;
    lower_bound.block(bound_eq_matrix.rows() , 0 , lower_bound_ineq_matrix.rows() , 1) = lower_bound_ineq_matrix;

    Eigen::VectorXd observe_weight_vector =
            Eigen::kroneckerProduct(Eigen::VectorXd::Constant(Np + 1 , 1) , parameters.observe_weight_vector);
    Eigen::MatrixXd observe_weight_matrix = observe_weight_vector.asDiagonal();
    observe_weight_matrix.block(0 , 0 , Nx + Nu , Nx + Nu) = Eigen::MatrixXd::Zero(Nx + Nu , Nx + Nu);
    // std::cout<<observe_weight_matrix<<std::endl;

    Eigen::VectorXd solution_weight_vector =
            Eigen::kroneckerProduct(Eigen::VectorXd::Constant(Np , 1) , parameters.solution_weight_vector);
    Eigen::MatrixXd solution_weight_matrix = solution_weight_vector.asDiagonal();
    // std::cout<<solution_weight_matrix<<std::endl;

    Eigen::MatrixXd quadprog_matrix_H = Eigen::MatrixXd::Zero((Np + 1)*(Nx + Nu) + Np*Nu , (Np + 1)*(Nx + Nu) + Np*Nu);
    quadprog_matrix_H.block(0 , 0 , observe_weight_matrix.rows() , observe_weight_matrix.cols()) = observe_weight_matrix;
    quadprog_matrix_H.block(observe_weight_matrix.rows() , observe_weight_matrix.cols() ,
                            solution_weight_matrix.rows() , solution_weight_matrix.cols()) =
            solution_weight_matrix;

    Eigen::MatrixXd multiple_erro = Eigen::MatrixXd::Zero((Np + 1)*(Nx + Nu) + Np*Nu , 1);
    multiple_erro.block(0 , 0 , Nx , 1) = state_0;
    multiple_erro.block(Nx , 0 , Nu , 1) = control_n1;
    multiple_erro.block(Nx + Nu , 0 , Np*(Nx + Nu) , 1) = observe_reference;
    multiple_erro.block((Np + 1)*(Nx + Nu) , 0 , Np*Nu , 1) = solution_reference;

    Eigen::MatrixXd quadprog_matrix_f;
    quadprog_matrix_f =
            -multiple_erro.transpose()*quadprog_matrix_H;

    endTime = clock();

//    std::cout << "buliding problem cost: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;

    //求解二次规划问题

    startTime = clock();

    solver->clearSolver();

    solver->data()->clearHessianMatrix();
    solver->data()->clearLinearConstraintsMatrix();

    solver->settings()->setWarmStart(true);
    solver->settings()->setVerbosity(false);

    solver->data()->setNumberOfConstraints(constraint_matrix.rows());
    solver->data()->setNumberOfVariables(constraint_matrix.cols());

    Eigen::VectorXd lower_bound_vector = lower_bound;
    solver->data()->setLowerBound(lower_bound_vector);

    Eigen::VectorXd upper_bound_vector = upper_bound;
    solver->data()->setUpperBound(upper_bound_vector);

    Eigen::SparseMatrix<double> constraint_sparse_matrix;
    constraint_sparse_matrix = constraint_matrix.sparseView();
    solver->data()->setLinearConstraintsMatrix(constraint_sparse_matrix);

    Eigen::SparseMatrix<double> quadprog_sparse_matrix_H;
    quadprog_sparse_matrix_H = quadprog_matrix_H.sparseView();
    solver->data()->setHessianMatrix(quadprog_sparse_matrix_H);

    Eigen::VectorXd quadprog_vector_f =quadprog_matrix_f.transpose();
    solver->data()->setGradient( quadprog_vector_f);

    solver->initSolver();

    solver->solveProblem();

    Eigen::MatrixXd full_solution = solver->getSolution();

//    std::cout<<"\nH = \n"<<quadprog_sparse_matrix_H<<std::endl;

//    std::cout<<"\nf = \n"<<quadprog_vector_f<<std::endl;

//    std::cout<<"\ncon = \n"<<constraint_sparse_matrix<<std::endl;

//    std::cout<<"\nup = \n"<<upper_bound_vector<<std::endl;

//    std::cout<<"\nlow = \n"<<lower_bound_vector<<std::endl;

//    std::cout<<"\nfull_solution = \n"<<full_solution<<std::endl;

    solution = solver->getSolution().block((Np + 1)*(Nx + Nu) , 0 , Np*Nu , 1);

    solution.resize(Np*Nu,1);

    observe_optmize = full_solution.block(Nx + Nu , 0 , Np*(Nx + Nu) , 1);

    observe_optmize.resize(Nx + Nu , Np);

    endTime = clock();

//    std::cout << "optmizing cost: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;
}
