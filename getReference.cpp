#include    "LinearMPC_header.h"

bool LinearMPC_app::getReference(
    Eigen::MatrixXd state
){
    int Nx = parameters.Nx;

    int Nu = parameters.Nu;

    int Np = parameters.Np;

    double stepTime = parameters.stepTime;

    Eigen::MatrixXd t_vector{Np,1};

    for (auto i = 0; i < Np; i++)
    {
        t_vector(i,0) = (i + 1)*stepTime;
    }

    double v_ref = parameters.v_ref;

    Eigen::MatrixXd x_ref_vector{Np,1};

    Eigen::MatrixXd y_ref_vector{Np,1};

    Eigen::MatrixXd phi_ref_vector{Np,1};

    int mode = 4;

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
    else if (mode == 4)//跟踪通过话题发布的全局路径
    {
        //清空队列
        while (!distance_queue.empty())
        {
            distance_queue.pop();
        }

        if(ref_path_msg.poses.size() <= 1)
        {
            return false;
        }

        int n = 0;

        distance_node node;

        //遍历所有点，找到离自己最近的，目前这里在长路径规划时计算消耗有点大
        for(auto i:ref_path_msg.poses)
        {
            node.distance = sqrt(pow(i.pose.position.x - state(0,0),2) + pow(i.pose.position.y - state(1,0),2));

            node.sequence = n;

            distance_queue.push(node);

            n = n + 1;
        }

        distance_node top_distance_node = distance_queue.top();

        distance_queue.pop();

        distance_node second_distance_node = distance_queue.top();

        //找到离自己最近的两个点，并找出序列号较小的那个
        if(top_distance_node.sequence < second_distance_node.sequence)
        {
            n = top_distance_node.sequence;
        }
        else
        {
            n = second_distance_node.sequence;
        }

        //开始计算起始点，并将起点赋值给参考向量
        geometry_msgs::Pose startPose;

        geometry_msgs::Pose activePose;

//        geometry_msgs::Pose endPose;

        double a = sqrt(pow(ref_path_msg.poses[n].pose.position.x - state(0,0),2) +
                        pow(ref_path_msg.poses[n].pose.position.y - state(1,0),2));

        double b = sqrt(pow(ref_path_msg.poses[n].pose.position.x - ref_path_msg.poses[n + 1].pose.position.x,2) +
                        pow(ref_path_msg.poses[n].pose.position.y - ref_path_msg.poses[n + 1].pose.position.y,2));

        double c = sqrt(pow(ref_path_msg.poses[n + 1].pose.position.x - state(0,0),2) +
                        pow(ref_path_msg.poses[n + 1].pose.position.y - state(1,0),2));

        double theta = acos((pow(a,2) + pow(b,2) - pow(c,2))/(2*a*b));

        double alpha = a*cos(theta)/b;

        startPose.position.x = alpha*ref_path_msg.poses[n + 1].pose.position.x + (1 - alpha)*ref_path_msg.poses[n].pose.position.x;
        startPose.position.y = alpha*ref_path_msg.poses[n + 1].pose.position.y + (1 - alpha)*ref_path_msg.poses[n].pose.position.y;

        double phi = atan2(ref_path_msg.poses[n + 1].pose.position.y - ref_path_msg.poses[n].pose.position.y,
                ref_path_msg.poses[n + 1].pose.position.x - ref_path_msg.poses[n].pose.position.x);

        x_ref_vector(0,0) = startPose.position.x;
        y_ref_vector(0,0) = startPose.position.y;
        phi_ref_vector(0,0) = phi;

        //移动的点从起点开始，每次移动一个参考距离直到达到参考路径的终点。
        activePose = startPose;

        double restDistance;

        for(int i = 0;i < Np;i++)
        {
            if(n < ref_path_msg.poses.size() - 1)
            {
                //需要移动的距离等于参考距离
                restDistance = v_ref*parameters.stepTime;

                //一直移动直到：移动距离等于参考距离或到达终点
                while (restDistance > 0)
                {
                    double stepDistance = sqrt(pow(activePose.position.x - ref_path_msg.poses[n + 1].pose.position.x,2) +
                            pow(activePose.position.y - ref_path_msg.poses[n + 1].pose.position.y,2));

                    if(stepDistance >= restDistance)//不需要拐弯
                    {
                        activePose.position.x = activePose.position.x + restDistance*cos(phi);
                        activePose.position.y = activePose.position.y + restDistance*sin(phi);

                        restDistance = 0;
                    }
                    else//需要拐弯
                    {
                        //移动到下一个点
                        n += 1;

                        activePose.position.x = ref_path_msg.poses[n].pose.position.x;

                        activePose.position.y = ref_path_msg.poses[n].pose.position.y;

                        //如果到达终点就弹出
                        if(n == ref_path_msg.poses.size() - 1)
                        {
                            break;
                        }

                        phi = atan2(ref_path_msg.poses[n + 1].pose.position.y - ref_path_msg.poses[n].pose.position.y,
                                        ref_path_msg.poses[n + 1].pose.position.x - ref_path_msg.poses[n].pose.position.x);

                        restDistance = restDistance - stepDistance;
                    }
                }
            }

            //给参考向量赋值
            x_ref_vector(i,0) = activePose.position.x;
            y_ref_vector(i,0) = activePose.position.y;
            phi_ref_vector(i,0) = phi;
        }
    }
    else
    {
        printf("未知参考路径");

        return false;
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

    return true;
}
