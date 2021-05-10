#ifndef SIMULATORRVIZROSMODULE_H
#define SIMULATORRVIZROSMODULE_H

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include <string>
#include <cstdlib>
#include <stdio.h>
#include "boost/ref.hpp"

using namespace visualization_msgs;

class DroneRvizDisplay
{
private:
    ros::Subscriber sub;

protected:
    ros::NodeHandle n;

public:
    
    void frameCallback(const geometry_msgs::PoseStamped &pose_euler);
    void makeMovingMarker(const tf::Vector3& position , std::string name, std::string frame_id_in);
    void ServerApplyChanges();
    void ServerReset();
    void ServerResetNew();
    void open(ros::NodeHandle &nIn);
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

};


#endif
