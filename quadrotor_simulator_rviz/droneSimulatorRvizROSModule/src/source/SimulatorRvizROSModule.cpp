#include "SimulatorRvizROSModule.h"

using namespace visualization_msgs;

// %Tag(Box)%
Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::CUBE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;
  marker.color.a = 1.0;

  return marker;
}

Marker makeSphere(InteractiveMarker &msg )
{
    Marker marker;
    marker.color.r = 0.3;
    marker.color.g = 0.3;
    marker.color.b = 0.3;
    marker.color.a = 1.0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.mesh_resource = "package://droneArchitectureRvizROSModule/meshes/quadrotor_4.dae";
    return marker;

}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}
// %EndTag(Box)%





// %Tag(frameCallback)%

void DroneRvizDisplay::frameCallback(const geometry_msgs::PoseStamped &msg)
{

  static uint32_t counter = 0;
  static tf::TransformBroadcaster br;

  tf::Transform t;
  //ROS_INFO("TF sent");
  ros::Time time = ros::Time::now();

  t.setOrigin(tf::Vector3(double(msg.pose.position.x),double(msg.pose.position.y),double(msg.pose.position.z)));
  t.setRotation(tf::Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w));
  br.sendTransform(tf::StampedTransform(t, time, "base_link",std::string("moving_frame")+'0'));
  
  counter++;
  return;
}
// %EndTag(frameCallback)%

// %Tag(Moving)%
void DroneRvizDisplay::makeMovingMarker( const tf::Vector3& position , std::string name, std::string frame_id_in)
{
  std::string frame_id = frame_id_in+'0';
  InteractiveMarker int_marker;
  int_marker.header.frame_id = frame_id;
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = name;
  int_marker.description = name;

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
  control.always_visible = true;
  control.markers.push_back( makeSphere(int_marker) );
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  ROS_INFO("Created Marker");

}
// %EndTag(Moving)%

void DroneRvizDisplay::open(ros::NodeHandle &nIn)
{
    n=nIn;
    ROS_INFO("Subscribed to Simulated Drone Pose at /droneID/self_localization/pose");
    sub = n.subscribe("self_localization/pose", 1, &DroneRvizDisplay::frameCallback, this);

}

void DroneRvizDisplay::ServerResetNew()
{
    ROS_INFO("reset new");
    server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );
    return;
}

void DroneRvizDisplay::ServerReset()
{
    ROS_INFO("reset");
    server.reset();
    return;
}

void DroneRvizDisplay::ServerApplyChanges()
{
   ROS_INFO("changes");
   server->applyChanges();
   return;
}
