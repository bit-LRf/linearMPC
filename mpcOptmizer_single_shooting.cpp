#include    "LinearMPC_header.h"

bool LinearMPC_app::mpcOptmizer_single_shooting(
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

    Eigen::MatrixXd state_matrix_A_incremental;

    Eigen::MatrixXd state_matrix_B_incremental;

    Eigen::MatrixXd state_matrix_C_incremental;

    Eigen::MatrixXd constraint_matrix;

    Eigen::MatrixXd upper_bound;

    Eigen::MatrixXd lower_bound;

    state_matrix_A_incremental = Eigen::MatrixXd::Zero(Np*(Nx + Nu) , Nx + Nu);

    state_matrix_B_incremental = Eigen::MatrixXd::Zero(Np*(Nx + Nu) , Nu);

    state_matrix_C_incremental = Eigen::MatrixXd::Zero(Np*(Nx + Nu) , 1);

    state_eq = state_0;

    for (size_t i = 0; i < Np; i++)
    {
        control_eq = control_eq_vector.block(Nu*i , 0 , Nu , 1);

        getDiscreteMatrix(state_eq , control_eq);

        state_eq = getNextState(state_eq , control_eq);

        state_matrix_A_incremental.block(i*(Nx + Nu) , 0 , Nx , Nx) = state_matrix_A_discrete;
        state_matrix_A_incremental.block(i*(Nx + Nu) , Nx , Nx , Nu) = state_matrix_B_discrete;
        state_matrix_A_incremental.block(i*(Nx + Nu) + Nx , Nx , Nu , Nu) = Eigen::MatrixXd::Identity(Nu , Nu);

        state_matrix_B_incremental.block(i*(Nx + Nu) , 0 , Nx , Nu) = state_matrix_B_discrete;
        state_matrix_B_incremental.block(i*(Nx + Nu) + Nx, 0 , Nu , Nu) = Eigen::MatrixXd::Identity(Nu , Nu);

        state_matrix_C_incremental.block(i*(Nx + Nu) , 0 , Nx , 1) = state_matrix_C_discrete;
    }

    endTime = clock();

//    std::cout << "building incremental matrix cost: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;

    //建立预测矩阵

    startTime = clock();

    Eigen::MatrixXd observer_matrix_H = parameters.observer_matrix_H;

    single_shooting_matrix_A = Eigen::MatrixXd::Zero(Np*(observer_matrix_H.rows()) , Nx + Nu);
    single_shooting_matrix_B = Eigen::MatrixXd::Zero(Np*(observer_matrix_H.rows()) , Np*(Nx + Nu));
    generalized_control_B = Eigen::MatrixXd::Zero(Np*(Nx + Nu) , Np*Nu);
    generalized_control_C = state_matrix_C_incremental;

    table_matrix_A = Eigen::MatrixXd::Zero(Np*(Nx + Nu) , Nx + Nu);
    table_matrix_A.block(0 , 0 , Nx + Nu , Nx + Nu) = state_matrix_A_incremental.block(0 , 0 , Nx + Nu , Nx + Nu);

    single_shooting_matrix_A.block(0 , 0 , observer_matrix_H.rows() , Nx + Nu) = observer_matrix_H*table_matrix_A.block(0 , 0 , Nx + Nu , Nx + Nu);
    single_shooting_matrix_B.block(0 , 0 , observer_matrix_H.rows() , Nx + Nu) = observer_matrix_H*Eigen::MatrixXd::Identity(Nx + Nu , Nx + Nu);

    generalized_control_B.block(0 , 0 , Nx + Nu , Nu) = state_matrix_B_incremental.block(0 , 0 , Nx + Nu , Nu);
    
    for (size_t i = 1; i < Np; i++)
    {
        table_matrix_A.block(i*(Nx + Nu) , 0 , Nx + Nu , Nx + Nu) = 
        state_matrix_A_incremental.block(i*(Nx + Nu) , 0 , Nx + Nu , Nx + Nu)*table_matrix_A.block((i - 1)*(Nx + Nu) , 0 , Nx + Nu , Nx + Nu);

        single_shooting_matrix_A.block(i*(observer_matrix_H.rows()) , 0 , observer_matrix_H.rows() , Nx + Nu) = 
        observer_matrix_H*table_matrix_A.block(i*(Nx + Nu) , 0 , Nx + Nu , Nx + Nu);

        for (size_t j = 0; j < Np; j++)
        {
           generalized_control_B.block(i*(Nx + Nu) , i*Nu , Nx + Nu , Nu) = state_matrix_B_incremental.block(i*(Nx + Nu) , 0 , Nx + Nu , Nu);

           if (i >= j)
           {
                single_shooting_matrix_B.block(i*(observer_matrix_H.rows()) , j*(Nx + Nu) , observer_matrix_H.rows() , Nx + Nu) = 
                observer_matrix_H*table_matrix_A.block(i*(Nx + Nu) , 0 , Nx + Nu , Nx + Nu)*table_matrix_A.block(j*(Nx + Nu) , 0 , Nx + Nu , Nx + Nu).inverse();

                // std::cout<<
                // single_shooting_matrix_B.block(i*(observer_matrix_H.rows()) , i*(Nx + Nu) , observer_matrix_H.rows() , Nx + Nu)
                // <<std::endl;  

                // std::cout<<"-------------------\n"<<std::endl;
                
                // std::cout<<
                // single_shooting_matrix_B.block(0 , 0 , 9 , 10)
                // <<std::endl;  
           }
        }
    }

    endTime = clock();

//    std::cout << "building single shooting matrix cost: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;
    // std::cout<<table_matrix_A<<std::endl;
    // std::cout<<single_shooting_matrix_A<<std::endl;
    // std::cout<<single_shooting_matrix_B<<std::endl;
    // std::cout<<generalized_control_B<<std::endl;
    // std::cout<<generalized_control_C<<std::endl;

    //建立约束和权重矩阵

    startTime = clock();

    constraint_matrix = Eigen::MatrixXd::Zero(2*Np*Nu , Np*Nu);

    constraint_matrix.block(0 , 0 , Np*Nu , Np*Nu) = accumulation_matrix;
    constraint_matrix.block(Np*Nu , 0 , Np*Nu , Np*Nu) = Eigen::MatrixXd::Identity(Np*Nu , Np*Nu);
  
    upper_bound = Eigen::MatrixXd::Zero(2*Np*Nu , 1);
    lower_bound = Eigen::MatrixXd::Zero(2*Np*Nu , 1);

    upper_bound.block(0 , 0 , Np*Nu , 1) = 
    Eigen::kroneckerProduct(Eigen::MatrixXd::Constant(Np , 1 , 1) , parameters.control_max) - control_n1_vector;

    upper_bound.block(Np*Nu , 0 , Np*Nu , 1) = 
    Eigen::kroneckerProduct(Eigen::MatrixXd::Constant(Np , 1 , 1) , parameters.delta_control_max);

    lower_bound.block(0 , 0 , Np*Nu , 1) = 
    Eigen::kroneckerProduct(Eigen::MatrixXd::Constant(Np , 1 , 1) , parameters.control_min) - control_n1_vector;

    lower_bound.block(Np*Nu , 0 , Np*Nu , 1) = 
    Eigen::kroneckerProduct(Eigen::MatrixXd::Constant(Np , 1 , 1) , parameters.delta_control_min);

    Eigen::VectorXd observe_weight_vector = 
    Eigen::kroneckerProduct(Eigen::VectorXd::Constant(Np , 1) , parameters.observe_weight_vector);
    Eigen::MatrixXd observe_weight_matrix = observe_weight_vector.asDiagonal();
    // std::cout<<observe_weight_matrix<<std::endl;

    Eigen::VectorXd solution_weight_vector = 
    Eigen::kroneckerProduct(Eigen::VectorXd::Constant(Np , 1) , parameters.solution_weight_vector);
    Eigen::MatrixXd solution_weight_matrix = solution_weight_vector.asDiagonal();
    // std::cout<<solution_weight_matrix<<std::endl;

    endTime = clock();

//    std::cout << "building bound vector and weight matrix cost: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;

    //建立二次规划问题

    startTime = clock();

    Eigen::SparseMatrix<double> generalized_control_B_sparse;
    generalized_control_B_sparse = generalized_control_B.sparseView();

    Eigen::SparseMatrix<double> single_shooting_matrix_A_sparse;
    single_shooting_matrix_A_sparse = single_shooting_matrix_A.sparseView();

    Eigen::SparseMatrix<double> single_shooting_matrix_B_sparse;
    single_shooting_matrix_B_sparse = single_shooting_matrix_B.sparseView();

    Eigen::SparseMatrix<double> observe_weight_matrix_sparse;
    observe_weight_matrix_sparse = observe_weight_matrix.sparseView();

    Eigen::SparseMatrix<double> solution_weight_matrix_sparse;
    solution_weight_matrix_sparse = solution_weight_matrix.sparseView();

    Eigen::MatrixXd quadprog_matrix_H = 
    generalized_control_B_sparse.transpose()*single_shooting_matrix_B_sparse.transpose()*
    observe_weight_matrix_sparse*single_shooting_matrix_B_sparse*generalized_control_B_sparse
    + solution_weight_matrix_sparse;

    Eigen::MatrixXd single_shooting_const_state = Eigen::MatrixXd::Zero(Nx + Nu,1);
    single_shooting_const_state <<
    state_0,
    control_n1;

    Eigen::MatrixXd quadprog_erro = 
    single_shooting_matrix_A_sparse*single_shooting_const_state +
    single_shooting_matrix_B_sparse*generalized_control_C - observe_reference;
    
    Eigen::SparseMatrix<double> quadprog_erro_sparse;
    quadprog_erro_sparse = quadprog_erro.sparseView();

    Eigen::MatrixXd quadprog_matrix_f = 
    quadprog_erro_sparse.transpose()*observe_weight_matrix_sparse*
    single_shooting_matrix_B_sparse*generalized_control_B_sparse;

    endTime = clock();

//    std::cout << "building quadprog cost: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;

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

    solution = solver->getSolution();

    solution.resize(Np*Nu,1);

    observe_optmize =  single_shooting_matrix_A*single_shooting_const_state + 
    single_shooting_matrix_B*(generalized_control_B*solution + generalized_control_C);

    endTime = clock();

//    std::cout << "optmizing cost: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;
}
