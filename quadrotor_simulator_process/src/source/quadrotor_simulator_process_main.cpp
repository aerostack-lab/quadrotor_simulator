#include "quadrotor_simulator_process.h"

using namespace std;

int main(int argc,char **argv)
{
    //Ros Init
    ros::init(argc, argv, "quadrotor_simulator");
    ros::NodeHandle n;

    cout<<"[ROSNODE] Starting quadrotor_simulator"<<endl;

    //Vars
    QuadrotorSimulatorProcess quadrotor_simulator;
    quadrotor_simulator.setUp();
    quadrotor_simulator.start();

    ros::Rate loop_rate(quadrotor_simulator.get_moduleRate());

    try
    {
        while(ros::ok())
        {
            //Read messages
            ros::spinOnce();

            //run
            quadrotor_simulator.run();

            //Sleep
            loop_rate.sleep();

        }
    }
    catch (std::exception &ex)
    {
        std::cout<<"[ROSNODE] Exception :"<<ex.what()<<std::endl;
    }
}
