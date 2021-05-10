#include "SimulatorRvizROSModule.h"
#include <iostream>
#include "ros/ros.h"


// %Tag(main)%
int main(int argc, char** argv)
{

  ros::init(argc, argv, "basic_controls");
  ros::NodeHandle n;

  DroneRvizDisplay Drone;

  Drone.open(n);

  Drone.ServerResetNew();

  ros::Duration(0.1).sleep();

  tf::Vector3 position;

  position = tf::Vector3( 0, 0, 0);

  Drone.makeMovingMarker( position , "Simulated Drone", "moving_frame");
  Drone.ServerApplyChanges();

  ros::spin();

  Drone.ServerReset();
}
// %EndTag(main)%
