#include    "LinearMPC_header.h"

int main(int argc, char*  argv[])
{
    clock_t startTime,endTime;

    setlocale(LC_ALL, "");

    ros::init(argc,argv, "mpc_controler_ver1a");
    
    ros::NodeHandle nh;

    std::cout<<"节点启动\n-----------------------"<<std::endl;

    LinearMPC_app m_mpc(nh);

    int freqeuncy = int(1/m_mpc.parameters.stepTime);

    ros::Rate rate(freqeuncy);

    while (ros::ok())
    {
        startTime = clock();

        m_mpc.mpcUpdates();

        endTime = clock();

        std::cout << "cycle once cost: " << (double)(endTime - startTime) / CLOCKS_PER_SEC << "s" << std::endl;

        rate.sleep();

        m_mpc.mpcOutputs();
    }
}
