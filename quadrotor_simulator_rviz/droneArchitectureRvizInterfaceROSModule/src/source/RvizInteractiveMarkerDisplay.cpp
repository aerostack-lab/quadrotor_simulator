#include "RvizInteractiveMarkerDisplay.h"

#define SHOW_AREA 0

using namespace visualization_msgs;
using namespace interactive_markers;

//Menu Initialize
MenuHandler menu_handler;
MenuHandler::EntryHandle h_first_entry;
MenuHandler::EntryHandle h_mode_last=2;
MenuHandler::EntryHandle sub_menu_handle = menu_handler.insert( "Drone Selection" );

//Server Initialize
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

//Marker objects
visualization_msgs::Marker obstacles;
visualization_msgs::Marker obstacles_pole;
visualization_msgs::Marker walls;
visualization_msgs::Marker line_strip;
visualization_msgs::Marker point;

//Initialize Markers
int ini = 0;
int iniMP = 0;
int iniO = 0;
//int droneId;

//Number of subareas explored
//struct ID{
//  int id;
//  bool duplicated;
//};
int nsub_areas = 0;
int countedDrones=0;
ros::Time previous_time;
std::vector<int> DronesIDs;

//-------------------------Create Markers(Box / Sphere)------------------------------------------------//


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
//    marker.type = Marker::CUBE;
//    marker.scale.x = msg.scale * 0.4;
//    marker.scale.y = msg.scale * 0.4;
//    marker.scale.z = msg.scale * 0.4;
    marker.color.r = 0.3;
    marker.color.g = 0.3;
    marker.color.b = 0.3;
    marker.color.a = 1.0;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
//    marker.mesh_use_embedded_materials = true;
    marker.mesh_resource = "package://droneArchitectureRvizROSModule/meshes/quadrotor_4.dae";
    return marker;

}

Marker makeArea(InteractiveMarker &msg )
{

    Marker marker;
//    marker.type = Marker::CUBE;
//    marker.scale.x = msg.scale * 1;
//    marker.scale.y = msg.scale * 1;
//    marker.scale.z = msg.scale * 1;
    marker.color.r = 0.3;
    marker.color.g = 0.9    ;
    marker.color.b = 0.3;
    marker.color.a = 0.2;
//    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
//    marker.mesh_use_embedded_materials = true;
    marker.mesh_resource = "package://droneArchitectureRvizROSModule/meshes/stereo_pyramid_projection.dae";
    return marker;

}

Marker makeStereoPyramid(InteractiveMarker &msg )
{

    Marker marker;
//    marker.type = Marker::CUBE;
//    marker.scale.x = msg.scale * 0.4;
//    marker.scale.y = msg.scale * 0.4;
//    marker.scale.z = msg.scale * 0.4;
    marker.color.r = 0.3;
    marker.color.g = 0.9;
    marker.color.b = 0.3;
    marker.color.a = 0.3;
    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
//    marker.mesh_use_embedded_materials = true;
    marker.mesh_resource = "package://droneArchitectureRvizROSModule/meshes/stereo_pyramid.dae";
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

InteractiveMarker makeEmptyMarker( bool dummyBox=true )
{

    InteractiveMarker int_marker;
    int_marker.header.frame_id = "/base_link";             //Para cambiar la orientacion del GRF aqui se puede!
    int_marker.scale = 1;

    return int_marker;

}

void modeCb( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{

    menu_handler.setCheckState( h_mode_last, MenuHandler::UNCHECKED );
    h_mode_last = feedback->menu_entry_id;
    //droneId = h_mode_last-1;
    std::string namesss=feedback->control_name;
    ROS_INFO("Name: %s", namesss.c_str());
    ROS_INFO("last %i", h_mode_last);
    menu_handler.setCheckState( h_mode_last, MenuHandler::CHECKED );
    menu_handler.reApply( *server );
    server->applyChanges();


}


//--------------------------Create Menu/Interactive Markers---------------------------------------------//


void DroneRvizDisplay::initMenu(int i)
{

    std::ostringstream s;

    //droneId = i;
    s << "Drone " << i;

    h_mode_last = menu_handler.insert( sub_menu_handle, s.str(), &modeCb );
    menu_handler.setCheckState( h_mode_last, MenuHandler::UNCHECKED );
    h_mode_last = 2;
    menu_handler.setCheckState( h_mode_last, MenuHandler::CHECKED );  //try
    //

    ROS_INFO("INIT");
    //obstacles.ns="zero";


}

void DroneRvizDisplay::makeAxesMenu(std::string name)
{

    InteractiveMarker int_marker = makeEmptyMarker();
    int_marker.name = name;
    int_marker.description = name;
    InteractiveMarkerControl control;
    control.interaction_mode = InteractiveMarkerControl::BUTTON;
    control.markers.push_back(makeBox(int_marker));
    control.always_visible = true;
    int_marker.controls.push_back(control);
    server->insert(int_marker);

}

void DroneRvizDisplay::makeMovingMarker( const tf::Vector3& position , std::string name, std::string frame_id_in)
{

    /* Drone creation */
    std::string frame_id = std::string("Moving ") + std::string(frame_id_in);;
    InteractiveMarker int_marker;
    int_marker.header.frame_id = frame_id;

    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 1;

    ROS_INFO("%s", name.c_str());

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

    static tf::TransformBroadcaster br;
    tf::Transform t;
    ros::Time time = ros::Time::now();

    t.setOrigin(tf::Vector3(1.0,8.0,0.0));
    t.setRotation(tf::createQuaternionFromRPY(0.0,0.0,0.0));
    br.sendTransform(tf::StampedTransform(t, time, "base_link",frame_id_in));



    if (SHOW_AREA){
        /* Stereo pyramid creation */
        std::string frame_id_pyra = std::string("pyramid ") + std::string(frame_id_in);
        InteractiveMarker int_marker_pyra;
        int_marker_pyra.header.frame_id = frame_id_pyra;

        tf::pointTFToMsg(position, int_marker_pyra.pose.position);
        int_marker_pyra.scale = 1;

//        ROS_INFO("%s", name.c_str());

        int_marker_pyra.name = frame_id_pyra;
        int_marker_pyra.description = frame_id_pyra;
        InteractiveMarkerControl control_pyra;
        control_pyra.orientation.w = 1;
        control_pyra.orientation.x = 1;
        control_pyra.orientation.y = 0;
        control_pyra.orientation.z = 0;
        control_pyra.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker_pyra.controls.push_back(control_pyra);
        control_pyra.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
        control_pyra.always_visible = true;
        control_pyra.markers.push_back( makeStereoPyramid(int_marker_pyra) );
        int_marker_pyra.controls.push_back(control_pyra);

        server->insert(int_marker_pyra);

        ROS_INFO("Created Stereo Pyramid Marker");

        static tf::TransformBroadcaster br_pyra;
        tf::Transform t_pyra;
        ros::Time time_pyra = ros::Time::now();

        t_pyra.setOrigin(tf::Vector3(1.0,8.0,0.0));
        t_pyra.setRotation(tf::createQuaternionFromRPY(0.0,0.0,0.0));
        br_pyra.sendTransform(tf::StampedTransform(t_pyra, time_pyra, "base_link",frame_id_pyra));
    }
}


//--------------------------SERVER FUNCTIONS-------------------------------------------------------------//


void DroneRvizDisplay::ServerReset()
{

    server.reset();

    return;

}

void DroneRvizDisplay::setDroneId(int droneId)
{
    idDrone = droneId;
    return;
}

int DroneRvizDisplay::ActiveDroneId()
{
  return h_mode_last-1;
}

void DroneRvizDisplay::ServerResetNew()
{

    server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );

    return;

}

void DroneRvizDisplay::MenuHandlerApply()
{

    menu_handler.apply(*server, "GRF");

}

void DroneRvizDisplay::ServerApplyChanges()
{

    server->applyChanges();

    return;

}


//---------------------------SUBSCRIBER CALLBACK FUNCTIONS--------------------------------------------------//


void DroneRvizDisplay::PoseCallback(const droneMsgsROS::dronePose &pose_euler, int idDrone)
{

    static tf::TransformBroadcaster br;
    tf::Transform t;
    ros::Time time = ros::Time::now();

    t.setOrigin(tf::Vector3(double(pose_euler.x),double(pose_euler.y),double(pose_euler.z)));
    t.setRotation(tf::createQuaternionFromRPY(pose_euler.roll, pose_euler.pitch, pose_euler.yaw));
    br.sendTransform(tf::StampedTransform(t, time, "base_link",std::string("Moving ")+cvg_int_to_string(idDrone)));
    std::string frame=std::string("Moving ") + cvg_int_to_string(idDrone);

    //ROS_INFO("%s",frame.c_str());
    if (SHOW_AREA){
        static tf::TransformBroadcaster br_pyra;
        tf::Transform t_pyra;
        ros::Time time_pyra = ros::Time::now();

        t_pyra.setOrigin(tf::Vector3(double(pose_euler.x),double(pose_euler.y),double(pose_euler.z)));
        t_pyra.setRotation(tf::createQuaternionFromRPY(pose_euler.roll, pose_euler.pitch, pose_euler.yaw));
        br_pyra.sendTransform(tf::StampedTransform(t_pyra, time_pyra, "base_link",std::string("pyramid ") + cvg_int_to_string(idDrone)));
    }


    /* Looking for the ID */
    bool found = false;
    for(std::vector<int>::iterator it = DronesIDs.begin(); it != DronesIDs.end(); ++it) {
        if(*it == idDrone){
            found = true;
        }
    }
    /* If not found, insert it */
    if(!found)  DronesIDs.push_back(idDrone);

//    std::cout << "Number of drones: " << DronesIDs.size() << std::endl;

    /* Explored Area Tracking (projection) */
    if(SHOW_AREA && ((nsub_areas % 30) == 0)){
        /* Re-asignment of time */
        countedDrones++;
        if (countedDrones == DronesIDs.size()) countedDrones = 0;
        else    nsub_areas = nsub_areas - 1;

        /* Projection of the explored area on the floor */
        std::ostringstream oss;
        oss << "area" << nsub_areas << idDrone;
        std::string frame_id = oss.str();
        InteractiveMarker int_marker;
        int_marker.header.frame_id = frame_id;



        tf::pointTFToMsg(tf::Vector3(0.0,0.0,0.0), int_marker.pose.position);
        int_marker.scale = 1;

        //ROS_INFO("%s", frame_id);

        int_marker.name = frame_id;
        int_marker.description = frame_id;
        InteractiveMarkerControl control;
        control.orientation.w = 0;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 0;
//        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
//        int_marker.controls.push_back(control);
//        control.interaction_mode = InteractiveMarkerControl::MOVE_PLANE;
        control.always_visible = false;
        control.markers.push_back( makeArea(int_marker) );
        int_marker.controls.push_back(control);

        server->insert(int_marker);
        server->applyChanges();

        ROS_INFO("Created Area Marker");

        static tf::TransformBroadcaster br_area;
        tf::Transform t_area;
        ros::Time time_area = ros::Time::now();

        t_area.setOrigin(tf::Vector3(double(pose_euler.x),double(pose_euler.y), 0));
        t_area.setRotation(tf::createQuaternionFromRPY(pose_euler.roll, pose_euler.pitch, pose_euler.yaw));
        br_area.sendTransform(tf::StampedTransform(t_area, time_area, "base_link",frame_id));
    }

    nsub_areas++;

    return;

}

void DroneRvizDisplay::ObstaclesPubCallback(const droneMsgsROS::obstaclesTwoDim obstacles, int idDrone)
{

    if (iniO == idDrone)
    {
        DroneRvizDisplay::ObstaclesPubCallbackAdd(obstacles, "Obstacles Drone"+cvg_int_to_string(idDrone));
    }
    else if (iniO==0)
    {
        DroneRvizDisplay::ObstaclesPubCallbackAdd(obstacles, "Obstacles Drone"+cvg_int_to_string(idDrone));
        iniO=idDrone;
    }
    else if (iniO != idDrone && iniO != 0)
    {
        DroneRvizDisplay::ObstaclesPubCallbackDelete(obstacles, "Obstacles Drone"+cvg_int_to_string(iniO));
        DroneRvizDisplay::ObstaclesPubCallbackAdd(obstacles, "Obstacles Drone"+cvg_int_to_string(idDrone));
        iniO=idDrone;

    }

}

void DroneRvizDisplay::TrajectoryPubCallback(const droneMsgsROS::dronePositionTrajectoryRefCommand trajectory, int idDrone)
{

    if (ini == idDrone)
    {
        DroneRvizDisplay::TrajectoryPubCallbackAdd(trajectory, idDrone, "Trajectory Drone"+cvg_int_to_string(idDrone));
    }
    else if (ini==0)
    {
        DroneRvizDisplay::TrajectoryPubCallbackAdd(trajectory, idDrone, "Trajectory Drone"+cvg_int_to_string(idDrone));
        ini=idDrone;
    }
    else if (ini != idDrone && ini != 0)
    {
        DroneRvizDisplay::TrajectoryPubCallbackDelete(trajectory, ini, "Trajectory Drone"+cvg_int_to_string(ini));
        DroneRvizDisplay::TrajectoryPubCallbackAdd(trajectory, idDrone, "Trajectory Drone"+cvg_int_to_string(idDrone));
        ini=idDrone;

    }

}

void DroneRvizDisplay::MissionPointPubCallback(const droneMsgsROS::dronePositionTrajectoryRefCommand missionPoint, int idDrone)
{

    if (iniMP == idDrone)
    {
        DroneRvizDisplay::MissionPointPubCallbackAdd(missionPoint, idDrone, "Mission Point Drone"+cvg_int_to_string(idDrone));
    }
    else if (iniMP==0)
    {
        DroneRvizDisplay::MissionPointPubCallbackAdd(missionPoint, idDrone, "Mission Point Drone"+cvg_int_to_string(idDrone));
        iniMP=idDrone;
    }
    else if (iniMP != idDrone && iniMP != 0)
    {
        DroneRvizDisplay::MissionPointPubCallbackDelete(missionPoint, iniMP, "Mission Point Drone"+cvg_int_to_string(iniMP));
        DroneRvizDisplay::MissionPointPubCallbackAdd(missionPoint, idDrone, "Mission Point Drone"+cvg_int_to_string(idDrone));
        iniMP=idDrone;

    }

}


//----------------------------ADD/DELETE MARKERS CALLBACKS--------------------------------------------------//


void DroneRvizDisplay::ObstaclesPubCallbackAdd(const droneMsgsROS::obstaclesTwoDim Drone_obstacles, std::string name)
{

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    obstacles.header.frame_id = "/base_link";
    obstacles.header.stamp = ros::Time::now();
    obstacles_pole.header.frame_id = "/base_link";
    obstacles_pole.header.stamp = ros::Time::now();
    walls.header.frame_id = "/base_link";
    walls.header.stamp = ros::Time::now();

    // Set the namespace and id for this obstacles.  This serves to create a unique ID
    // Any obstacles sent with the same namespace and id will overwrite the old one
    obstacles.ns=name;
    obstacles_pole.ns=name;
    walls.ns = name;
    // Set the obstacles type.
//    obstacles.type = visualization_msgs::Marker::CYLINDER;
    obstacles.type = visualization_msgs::Marker::MESH_RESOURCE;
    obstacles.mesh_use_embedded_materials = true;

    walls.type = visualization_msgs::Marker::CUBE;
    obstacles_pole.type = visualization_msgs::Marker::CYLINDER;

    // Set the pose of the obstacles.  This is a full 6DOF pose relative to the frame/time specified in the header
    obstacles.pose.orientation.x = 0.0;
    obstacles.pose.orientation.y = 0.0;
    obstacles.pose.orientation.z = 0.0;
    obstacles.pose.orientation.w = 1.0;

    obstacles_pole.pose.orientation.x = 0.0;
    obstacles_pole.pose.orientation.y = 0.0;
    obstacles_pole.pose.orientation.z = 0.0;
    obstacles_pole.pose.orientation.w = 1.0;

    // Set the pose of the obstacles.  This is a full 6DOF pose relative to the frame/time specified in the header
    walls.pose.orientation.x = 0.0;
    walls.pose.orientation.y = 0.0;
    walls.pose.orientation.z = 0.0;
    walls.pose.orientation.w = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
//    obstacles.color.r = 0.0f;
//    obstacles.color.g = 0.0f;
//    obstacles.color.b = 0.8f;
//    obstacles.color.a = 1.0;

    obstacles_pole.color.r = 0.823529f;
    obstacles_pole.color.g = 0.411765f;
    obstacles_pole.color.b = 0.117647f;
    obstacles_pole.color.a = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    walls.color.r = 0.9f;
    walls.color.g = 0.0f;
    walls.color.b = 0.0f;
    walls.color.a = 1.0;

    // Marker Lifetime
    obstacles.lifetime = ros::Duration();
    obstacles_pole.lifetime = ros::Duration();

    // Marker Lifetime
    walls.lifetime = ros::Duration();

    //Obstacle Iterator

    for ( std::vector<droneMsgsROS::obstacleTwoDimPole>::const_iterator it = Drone_obstacles.poles.begin();
          it != Drone_obstacles.poles.end();
          ++it) {

        obstacles_pub = n.advertise<visualization_msgs::Marker>("visualization_obstacles", 1000);

        id_O = it->id;
        x1 = it->centerX;
        y = it->centerY;
        rx = it->radiusX;
        ry = it->radiusY;
        yaw = it->yawAngle;

        if((id_O / 10000) == 6){
            obstacles.mesh_resource = "package://droneArchitectureRvizROSModule/meshes/aruco_landmark.dae";
            if ((x1 >= -0.5) && ( x1 <= 0.5)){
                obstacles.pose.orientation.z = -M_PI/8;
            }
            else if((y >= 9.5) && (y <= 10.5)){
                obstacles.pose.orientation.z = -5*M_PI/8;
            }
            else{
                obstacles.pose.orientation.z = 5*M_PI/8;
            }

        }
        else{
            obstacles.mesh_resource = "package://droneArchitectureRvizROSModule/meshes/aruco.dae";
        }

        //Unique Obstacle ID
        obstacles.id = id_O;
//        std::cout << "obstacle id: " << id_O << std::endl;
        obstacles_pole.id = id_O*100;

        // Set the obstacles action
        obstacles.action = visualization_msgs::Marker::ADD;
        obstacles_pole.action = visualization_msgs::Marker::ADD;

        //Set Obstacle Pose
        obstacles.pose.position.x = x1;
        obstacles.pose.position.y = y;
//        std::cout << "id: " << id_O << "x: " << x1 << "y: " << y << std::endl;
//        obstacles.pose.position.z = 1.0;
        obstacles.pose.position.z = 1.2;

        obstacles_pole.pose.position.x = x1;
        obstacles_pole.pose.position.y = y;
        obstacles_pole.pose.position.z = 0.6;

        //Set Obstacle Scale
//        obstacles.scale.x = rx;
//        obstacles.scale.y = ry;
//        obstacles.scale.z = 2.0;
        obstacles.scale.x = 1.0;
        obstacles.scale.y = 1.0;
        obstacles.scale.z = 1.0;

        obstacles_pole.scale.x = rx;
        obstacles_pole.scale.y = ry;
        obstacles_pole.scale.z = 1.2;

        // Publish the obstacles
        obstacles_pub.publish(obstacles_pole);
        obstacles_pub.publish(obstacles);

    }

    //Walls Iterator

    for (std::vector<droneMsgsROS::obstacleTwoDimWall>::const_iterator it = Drone_obstacles.walls.begin();
         it != Drone_obstacles.walls.end();
         ++it ) {



        obstacles_pub = n.advertise<visualization_msgs::Marker>("visualization_obstacles", 1000);

        id_2 = it->id;
        x2 = it->centerX;
        y2 = it->centerY;
        sx = it->sizeX;
        sy = it->sizeY;
        yaw2 = it->yawAngle;

        //Unique Obstacle ID
        walls.id = id_2;

        // Set the obstacles action
        walls.action = visualization_msgs::Marker::ADD;

        //Set Obstacle Pose
        walls.pose.position.x = x2;
        walls.pose.position.y = y2;
        walls.pose.position.z = 0.15;

        //Set Obstacle Scale
        walls.scale.x = sx;
        walls.scale.y = sy;
        walls.scale.z = 0.3;

        // Publish the obstacles
        obstacles_pub.publish(walls);

    }

}

void DroneRvizDisplay::ObstaclesPubCallbackDelete(const droneMsgsROS::obstaclesTwoDim Drone_obstacles, std::string name)
{

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    obstacles.header.frame_id = "/base_link";
    obstacles.header.stamp = ros::Time::now();
    walls.header.frame_id = "/base_link";
    walls.header.stamp = ros::Time::now();

    // Set the namespace and id for this obstacles.  This serves to create a unique ID
    // Any obstacles sent with the same namespace and id will overwrite the old one
    obstacles.ns=name;
    walls.ns = name;
    // Set the obstacles type.
    obstacles.type = visualization_msgs::Marker::CYLINDER;
    walls.type = visualization_msgs::Marker::CUBE;

    // Set the pose of the obstacles.  This is a full 6DOF pose relative to the frame/time specified in the header
    obstacles.pose.orientation.x = 0.0;
    obstacles.pose.orientation.y = 0.0;
    obstacles.pose.orientation.z = 0.0;
    obstacles.pose.orientation.w = 1.0;

    // Set the pose of the obstacles.  This is a full 6DOF pose relative to the frame/time specified in the header
    walls.pose.orientation.x = 0.0;
    walls.pose.orientation.y = 0.0;
    walls.pose.orientation.z = 0.0;
    walls.pose.orientation.w = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    obstacles.color.r = 0.0f;
    obstacles.color.g = 0.0f;
    obstacles.color.b = 0.8f;
    obstacles.color.a = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    walls.color.r = 0.9f;
    walls.color.g = 0.0f;
    walls.color.b = 0.0f;
    walls.color.a = 0.7;

    // Marker Lifetime
    obstacles.lifetime = ros::Duration();

    // Marker Lifetime
    walls.lifetime = ros::Duration();

    //Obstacle Iterator


    for ( std::vector<droneMsgsROS::obstacleTwoDimPole>::const_iterator it = Drone_obstacles.poles.begin();
          it != Drone_obstacles.poles.end();
          ++it) {

        obstacles_pub = n.advertise<visualization_msgs::Marker>("visualization_obstacles", 1000);

        id_O = it->id;
        x1 = it->centerX;
        y = it->centerY;
        rx = it->radiusX;
        ry = it->radiusY;
        yaw = it->yawAngle;

        //Unique Obstacle ID
        obstacles.id = id_O;

        // Set the obstacles action
        obstacles.action = visualization_msgs::Marker::DELETE;

        //Set Obstacle Pose
        obstacles.pose.position.x = x1;
        obstacles.pose.position.y = y;
        obstacles.pose.position.z = 1.0;

        //Set Obstacle Scale
        obstacles.scale.x = rx;
        obstacles.scale.y = ry;
        obstacles.scale.z = 2.0;

        // Publish the obstacles
        obstacles_pub.publish(obstacles);

    }

    //Walls Iterator


    for (std::vector<droneMsgsROS::obstacleTwoDimWall>::const_iterator it = Drone_obstacles.walls.begin();
         it != Drone_obstacles.walls.end();
         ++it ) {

        obstacles_pub = n.advertise<visualization_msgs::Marker>("visualization_obstacles", 1000);

        id_2 = it->id;
        x2 = it->centerX;
        y2 = it->centerY;
        sx = it->sizeX;
        sy = it->sizeY;
        yaw2 = it->yawAngle;

        //Unique Obstacle ID
        walls.id = id_2;

        // Set the obstacles action
        walls.action = visualization_msgs::Marker::DELETE;

        //Set Obstacle Pose
        walls.pose.position.x = x2;
        walls.pose.position.y = y2;
        walls.pose.position.z = 1.0;

        //Set Obstacle Scale
        walls.scale.x = sx;
        walls.scale.y = sy;
        walls.scale.z = 2.0;

        // Publish the obstacles
        obstacles_pub.publish(walls);

    }

}

void DroneRvizDisplay::TrajectoryPubCallbackAdd(const droneMsgsROS::dronePositionTrajectoryRefCommand trajectory, int idDrone, std::string name)
{

    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "/base_link";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = name;
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;


    line_strip.id = idDrone;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.03;

    // Line strip is green
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;

    int size=trajectory.droneTrajectory.size();

    if (trajectory.droneTrajectory.size()>0)
    {
        for (int i=0; i < size; i++)
        {

            geometry_msgs::Point p;
            p.x = trajectory.droneTrajectory[i].x;
            p.y = trajectory.droneTrajectory[i].y;
            p.z = trajectory.droneTrajectory[i].z;

            line_strip.points.push_back(p);

        }

    }
    else if (trajectory.droneTrajectory.size()==0)
    {

        geometry_msgs::Point p;
        p.x = 0.0;
        p.y = 0.0;
        p.z = 0.0;

        line_strip.points.push_back(p);

    }
    //ros::NodeHandle n;

    trajectory_pub = n.advertise<visualization_msgs::Marker>("visualization_trajectory", 10);
    trajectory_pub.publish(line_strip);

}

void DroneRvizDisplay::TrajectoryPubCallbackDelete(const droneMsgsROS::dronePositionTrajectoryRefCommand trajectory, int idDrone, std::string name)
{

    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "/base_link";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = name;
    line_strip.action = visualization_msgs::Marker::DELETE;
    line_strip.pose.orientation.w = 1.0;


    line_strip.id = idDrone;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.03;

    // Line strip is blue
    line_strip.color.r = 1.0;
    line_strip.color.a = 1.0;

    int size=trajectory.droneTrajectory.size();

    if (trajectory.droneTrajectory.size()>0)
    {
        for (int i=0; i < size; i++)
        {

            geometry_msgs::Point p;
            p.x = trajectory.droneTrajectory[i].x;
            p.y = trajectory.droneTrajectory[i].y;
            p.z = trajectory.droneTrajectory[i].z;

            line_strip.points.push_back(p);

        }

    }
    else if (trajectory.droneTrajectory.size()==0)
    {

        geometry_msgs::Point p;
        p.x = 0.0;
        p.y = 0.0;
        p.z = 0.0;

        line_strip.points.push_back(p);

    }
    //ros::NodeHandle n;

    trajectory_pub = n.advertise<visualization_msgs::Marker>("visualization_trajectory", 10);
    trajectory_pub.publish(line_strip);


}

void DroneRvizDisplay::MissionPointPubCallbackAdd(const droneMsgsROS::dronePositionTrajectoryRefCommand missionPoint, int idDrone, std::string name)
{


    point.header.frame_id = "/base_link";
    point.header.stamp = ros::Time();
    point.ns = name;
    point.id = idDrone;
    point.type = visualization_msgs::Marker::SPHERE;
    point.action = visualization_msgs::Marker::ADD;

    point.pose.orientation.x = 0.0;
    point.pose.orientation.y = 0.0;
    point.pose.orientation.z = 0.0;
    point.pose.orientation.w = 1.0;

    if (missionPoint.droneTrajectory.size()>0)
    {
        point.scale.x = 0.2;
        point.scale.y = 0.2;
        point.scale.z = 0.2;

        point.pose.position.x = missionPoint.droneTrajectory[missionPoint.droneTrajectory.size()-1].x;
        point.pose.position.y = missionPoint.droneTrajectory[missionPoint.droneTrajectory.size()-1].y;
        point.pose.position.z = missionPoint.droneTrajectory[missionPoint.droneTrajectory.size()-1].z;

    }
    else if (missionPoint.droneTrajectory.size()==0)
    {
        point.scale.x = 0.0;
        point.scale.y = 0.0;
        point.scale.z = 0.0;

        point.pose.position.x = 0.0;
        point.pose.position.y = 0.0;
        point.pose.position.z = 0.0;

    }

    point.color.a = 1.0;
    point.color.r = 1.0;
    point.color.g = 0.0;
    point.color.b = 0.0;

    mission_pub = n.advertise<visualization_msgs::Marker>( "visualization_missionPoint", 100);
    mission_pub.publish( point );

}

void DroneRvizDisplay::MissionPointPubCallbackDelete(const droneMsgsROS::dronePositionTrajectoryRefCommand missionPoint, int idDrone, std::string name)
{

    visualization_msgs::Marker point;
    point.header.frame_id = "/base_link";
    point.header.stamp = ros::Time();
    point.ns = name;
    point.id = idDrone;
    point.type = visualization_msgs::Marker::SPHERE;
    point.action = visualization_msgs::Marker::DELETE;

    point.pose.orientation.x = 0.0;
    point.pose.orientation.y = 0.0;
    point.pose.orientation.z = 0.0;
    point.pose.orientation.w = 1.0;

    if (missionPoint.droneTrajectory.size()>0)
    {
        point.scale.x = 0.2;
        point.scale.y = 0.2;
        point.scale.z = 0.2;

        point.pose.position.x = missionPoint.droneTrajectory[missionPoint.droneTrajectory.size()-1].x;
        point.pose.position.y = missionPoint.droneTrajectory[missionPoint.droneTrajectory.size()-1].y;
        point.pose.position.z = missionPoint.droneTrajectory[missionPoint.droneTrajectory.size()-1].z;

    }
    else if (missionPoint.droneTrajectory.size()==0)
    {
        point.scale.x = 0.0;
        point.scale.y = 0.0;
        point.scale.z = 0.0;

        point.pose.position.x = 0.0;
        point.pose.position.y = 0.0;
        point.pose.position.z = 0.0;

    }

    point.color.a = 1.0;
    point.color.r = 1.0;
    point.color.g = 0.0;
    point.color.b = 0.0;

    mission_pub = n.advertise<visualization_msgs::Marker>( "visualization_missionPoint", 100);
    mission_pub.publish( point );

}
