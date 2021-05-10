#include "swarmAgentInterface.h"
#include "RvizInteractiveMarkerDisplay.h"



SwarmAgentInterface::SwarmAgentInterface(int idDrone_in)
{

    idDrone = idDrone_in;
    return;

}

SwarmAgentInterface::~SwarmAgentInterface()
{

    return;

}

void SwarmAgentInterface::open(ros::NodeHandle &nIn)
{

    n = nIn;

    localizer_pose_subscriber                          = n.subscribe(std::string("/drone")+cvg_int_to_string(idDrone)+"/"+"EstimatedPose_droneGMR_wrt_GFF", 1, &SwarmAgentInterface::localizerPoseCallback, this);
    mission_planner_mission_point_reference_subscriber = n.subscribe(std::string("/drone")+cvg_int_to_string(idDrone)+"/"+"trajectoryControllerTrajectoryReferencesRebroadcast", 1, &SwarmAgentInterface::missionPointCallback, this);
    trajectory_planner_trajectory_reference_subscriber = n.subscribe(std::string("/drone")+cvg_int_to_string(idDrone)+"/"+"trajectoryControllerTrajectoryReferencesRebroadcast", 1, &SwarmAgentInterface::trajectoryAbsRefCmdCallback, this);
    obstacle_processor_obstacle_list_subscriber        = n.subscribe(std::string("/drone")+cvg_int_to_string(idDrone)+"/"+"obstacles", 1, &SwarmAgentInterface::obstacleListCallback, this);
//    this_drone_society_pose_subscriber                 = n.subscribe(std::string("/drone")+cvg_int_to_string(idDrone)+"/"+DRONE_ARCHITECTURE_RVIZ_INTERFACE_SOCIETY_POSE_SUBSCRIPTION, 1, &SwarmAgentInterface::societyPoseSubCallback, this);

    return;

}



void SwarmAgentInterface::localizerPoseCallback(const droneMsgsROS::dronePose &pose_euler)
{

    Drone.PoseCallback(pose_euler, idDrone);

}

void SwarmAgentInterface::missionPointCallback(const droneMsgsROS::dronePositionTrajectoryRefCommand &point)
{

    if (idDrone == Drone.ActiveDroneId())
    {
        Drone.MissionPointPubCallback(point, idDrone);
    }

}

void SwarmAgentInterface::trajectoryAbsRefCmdCallback(const droneMsgsROS::dronePositionTrajectoryRefCommand &trajectory)
{


    if (idDrone == Drone.ActiveDroneId())
    {

        Drone.TrajectoryPubCallback(trajectory, idDrone);
    }

}

void SwarmAgentInterface::obstacleListCallback(const droneMsgsROS::obstaclesTwoDim &obstacles)
{

    //ROS_INFO("Active Drone ID 1 %i", Drone.ActiveDroneId());
    //ROS_INFO("Drone ID 2 %i", idDrone);
    if (idDrone == Drone.ActiveDroneId())
    {
        Drone.ObstaclesPubCallback(obstacles, idDrone);
    }

}

void SwarmAgentInterface::societyPoseSubCallback(const droneMsgsROS::societyPose::ConstPtr &msg)
{

    //std::cout << "SwarmAgentInterface::societyPoseSubCallback drone:" << idDrone << std::endl;

}




