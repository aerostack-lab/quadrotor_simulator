#include "quadrotor_simulator_process.h"

QuadrotorSimulatorProcess::QuadrotorSimulatorProcess(){
}

QuadrotorSimulatorProcess::~QuadrotorSimulatorProcess(){
}

void QuadrotorSimulatorProcess::ownSetUp()
{
    //Configs
    ros::param::get("~robot_namespace", robot_namespace);
    ros::param::get("~frequency", rate);
    ros::param::get("~robot_config_path", robot_config_path);
    ros::param::get("~config_file", configFile);

    //Topics
    ros::param::get("~estimated_speed", estimated_speed_topic);
    ros::param::get("~estimated_pose", estimated_pose_topic);
    ros::param::get("~altitude", altitude_topic);
    ros::param::get("~sensor_speed", sensor_speed_topic);
    ros::param::get("~roll_pitch",roll_pitch_topic);
    ros::param::get("~altitude_yaw_rate",altitude_yaw_rate_topic);
    ros::param::get("~flight_action",flight_action_topic);
    ros::param::get("~battery_state",battery_state_topic);
    ros::param::get("~magnetometer",magnetometer_topic);
    ros::param::get("~pressure",pressure_topic);
    ros::param::get("~temperature",temperature_topic);
    ros::param::get("~flight_state",flight_state_topic);

    quadrotor_simulator.init(robot_config_path+"/"+configFile);
    last_speed_time.sec = 0;
}

double QuadrotorSimulatorProcess::get_moduleRate(){
    return rate;
}

void QuadrotorSimulatorProcess::ownStart(){
    ros::NodeHandle n;

    //Subs
    roll_pitch_sub = n.subscribe(roll_pitch_topic, 1, &QuadrotorSimulatorProcess::rollPitchCallback, this);
    altitude_rate_yaw_rate_sub = n.subscribe(altitude_yaw_rate_topic, 1, &QuadrotorSimulatorProcess::altitudeRateYawRateCallback, this);
    flight_action_sub = n.subscribe(flight_action_topic, 1, &QuadrotorSimulatorProcess::flightActionCallback, this);    

    //Pubs
    estimated_pose_pub = n.advertise<geometry_msgs::PoseStamped>(estimated_pose_topic, 1, true);
    estimated_speed_pub = n.advertise<geometry_msgs::TwistStamped>(estimated_speed_topic, 1, true);
    altitude_pub = n.advertise<geometry_msgs::PointStamped>(altitude_topic, 1, true);
    sensor_speed_pub = n.advertise<geometry_msgs::TwistStamped>(sensor_speed_topic, 1, true);
    battery_state_pub = n.advertise<sensor_msgs::BatteryState>(battery_state_topic, 1, true);
    magnetometer_pub = n.advertise<geometry_msgs::Vector3Stamped>(magnetometer_topic, 1, true);
    pressure_pub = n.advertise<sensor_msgs::FluidPressure>(pressure_topic, 1, true);
    flight_state_pub = n.advertise<aerostack_msgs::FlightState>(flight_state_topic, 1, true);
    temperature_pub = n.advertise<sensor_msgs::Temperature>(temperature_topic, 1, true);
}

bool QuadrotorSimulatorProcess::resetValues(){
    return true;
}

void QuadrotorSimulatorProcess::ownStop(){
}

//Run
void QuadrotorSimulatorProcess::ownRun(){
    //Evaluate QuadrotorSimulator
    quadrotor_simulator.run();

    //Publish everything
    publishValues();
}

void QuadrotorSimulatorProcess::publishFlightState()
{
    aerostack_msgs::FlightState current_drone_status_msg;
    current_drone_status_msg.header.stamp=ros::Time::now();

    switch(quadrotor_simulator.getCurrentDroneState())
    {
    case DroneState::LANDED:
        current_drone_status_msg.state = aerostack_msgs::FlightState::LANDED;
        break;
    case DroneState::FLYING:
        current_drone_status_msg.state = aerostack_msgs::FlightState::FLYING;
        break;
    case DroneState::HOVERING:
        current_drone_status_msg.state = aerostack_msgs::FlightState::HOVERING;
        break;
    case DroneState::TAKING_OFF:
        current_drone_status_msg.state = aerostack_msgs::FlightState::TAKING_OFF;
        break;
    case DroneState::LANDING:
        current_drone_status_msg.state = aerostack_msgs::FlightState::LANDING;
        break;
    case DroneState::UNKNOWN:
    default:
        current_drone_status_msg.state = aerostack_msgs::FlightState::UNKNOWN;
        break;
    }

    //Publish
    flight_state_pub.publish(current_drone_status_msg);
}

void QuadrotorSimulatorProcess::rollPitchCallback(const geometry_msgs::PoseStamped& msg){
    double roll,pitch,y;
    toEulerianAngle(msg, &roll, &pitch, &y);

    //Pitch
    if(pitch > 1.0) pitch = 1.0;
    else if(pitch < -1.0) pitch = -1.0;

    //Roll
    roll = -roll;
    if(roll > 1.0) roll = 1.0;
    else if(roll < -1.0) roll = -1.0;

    quadrotor_simulator.DroneAutopilot.setPitchRollCommand(pitch,roll);
}

void QuadrotorSimulatorProcess::altitudeRateYawRateCallback(const geometry_msgs::TwistStamped& msg){
    double altitude_rate = msg.twist.linear.z;
    if(altitude_rate > 1.0) altitude_rate = 1.0;
    else if(altitude_rate < -1.0) altitude_rate = -1.0;

    double yaw_rate = -msg.twist.angular.z;
    if(yaw_rate > 1.0) yaw_rate = 1.0;
    else if(yaw_rate < -1.0) yaw_rate = -1.0;

    quadrotor_simulator.DroneAutopilot.setDYawCommand(yaw_rate);
    quadrotor_simulator.DroneAutopilot.setDAltitudeCommand(altitude_rate);
}

void QuadrotorSimulatorProcess::flightActionCallback(const aerostack_msgs::FlightActionCommand& msg){
    switch(msg.action)
    {
    case aerostack_msgs::FlightActionCommand::TAKE_OFF:
        quadrotor_simulator.commandDrone(DroneStateCommand::TAKE_OFF);
        break;
    case aerostack_msgs::FlightActionCommand::MOVE:
        quadrotor_simulator.commandDrone(DroneStateCommand::MOVE);
        break;
    case aerostack_msgs::FlightActionCommand::LAND:
        quadrotor_simulator.commandDrone(DroneStateCommand::LAND);
        break;
    case aerostack_msgs::FlightActionCommand::HOVER:
        quadrotor_simulator.commandDrone(DroneStateCommand::HOVER);
        break;
    case aerostack_msgs::FlightActionCommand::UNKNOWN:
    default:
        break;
    }
}

void QuadrotorSimulatorProcess::publishValues(){

//TEMPERATURE
    temperature_msg.header.stamp=ros::Time::now();
    quadrotor_simulator.DroneTermometer.getTemperature(temperature_msg.temperature,temperature_msg.variance);
    temperature_pub.publish(temperature_msg);

//MAGNETOMETER
    magnetometer_msg.header.stamp=ros::Time::now();
    quadrotor_simulator.DroneMagnetometer.getMagnetometer(magnetometer_msg.vector.x,magnetometer_msg.vector.y,magnetometer_msg.vector.z);
    magnetometer_pub.publish(magnetometer_msg);

//BATTERY
    battery_msg.header.stamp = ros::Time::now();
    battery_msg.percentage = quadrotor_simulator.DroneBattery.getPercentaje()/100;
    battery_state_pub.publish(battery_msg);

//ALTITUDE
    altitude_msg.header.stamp = ros::Time::now();
    double var_altitude;
    quadrotor_simulator.DroneAltitudeSensor.getAltitude(altitude_msg.point.z,var_altitude);
    //quadrotor_simulator.DroneAltitudeSensor.getAltitudeSpeed(AltitudeMsgs.altitude_speed,AltitudeMsgs.var_altitude_speed);
    altitude_pub.publish(altitude_msg);

//ROTATION ANGLES
    //RotationAnglesMsgs.header.stamp=ros::Time::now();
    //double rotation_x,rotation_y, rotation_z;
    //quadrotor_simulator.DroneRotationAnglesSensor.getRotationAngles(rotation_z,rotation_y,rotation_x);
    //std::cout << "ROTATION ANGLES: " << rotation_x <<", "<<rotation_y<<", "<<rotation_x<<std::endl;
    //RotationAnglesPubl.publish(RotationAnglesMsgs);

//SENSOR SPEED
    sensor_speed_msg.header.stamp=ros::Time::now();
    quadrotor_simulator.DroneGroundSpeedSensor.getGroundSpeed(sensor_speed_msg.twist.linear.x,sensor_speed_msg.twist.linear.y);
    sensor_speed_pub.publish(sensor_speed_msg);

//PRESSURE
    pressure_msg.header.stamp=ros::Time::now();
    quadrotor_simulator.DronePressureSensor.getPressure(pressure_msg.fluid_pressure,pressure_msg.variance);
    pressure_pub.publish(pressure_msg);

//Flight State
publishFlightState();

//POSE
    estimated_pose_msg.header.stamp=ros::Time::now();
    double x, y, z;
    quadrotor_simulator.getPosition_drone_GMR_wrt_GFF(x, y, z, pose_yaw, pose_pitch, pose_roll);

    tf::Quaternion quaternion = tf::createQuaternionFromRPY(pose_roll,pose_pitch,pose_yaw);

    estimated_pose_msg.pose.position.x = x;
    estimated_pose_msg.pose.position.y = y;
    estimated_pose_msg.pose.position.z = z;
    estimated_pose_msg.pose.orientation.x = quaternion.getX();
    estimated_pose_msg.pose.orientation.y = quaternion.getY();
    estimated_pose_msg.pose.orientation.z = quaternion.getZ();
    estimated_pose_msg.pose.orientation.w = quaternion.getW();

    estimated_pose_pub.publish(estimated_pose_msg);

//SPEED
    estimated_speed_msg.header.stamp = ros::Time::now();
    double dx, dy, dz;
    quadrotor_simulator.getSpeed_drone_GMR_wrt_GFF(dx, dy, dz);

    estimated_speed_msg.twist.linear.x = dx;
    estimated_speed_msg.twist.linear.y = dy;
    estimated_speed_msg.twist.linear.z = dz;

    if (last_speed_time.toSec() != 0){
        ros::Duration diff = estimated_speed_msg.header.stamp - last_speed_time;
        estimated_speed_msg.twist.angular.x = (pose_roll - last_roll)/diff.toSec();
        estimated_speed_msg.twist.angular.y = (pose_pitch - last_pitch)/diff.toSec();
        estimated_speed_msg.twist.angular.z = (pose_yaw - last_yaw)/diff.toSec();
    }   

    last_roll = pose_roll;
    last_pitch = pose_pitch;
    last_yaw = pose_yaw;
    last_speed_time = estimated_speed_msg.header.stamp;

    estimated_speed_pub.publish(estimated_speed_msg);
}

void QuadrotorSimulatorProcess::toEulerianAngle(geometry_msgs::PoseStamped q, double *roll, double *pitch, double *yaw){
    if(q.pose.orientation.w == 0 && q.pose.orientation.x == 0 && q.pose.orientation.y == 0 && q.pose.orientation.z == 0){
        *roll   = 0; 
        *pitch = 0;
        *yaw = 0;
    }else{
        *roll  = atan2(2.0 * (q.pose.orientation.z * q.pose.orientation.y + q.pose.orientation.w * q.pose.orientation.x) , 1.0 - 2.0 * (q.pose.orientation.x * q.pose.orientation.x + q.pose.orientation.y * q.pose.orientation.y));
        *pitch = asin(2.0 * (q.pose.orientation.y * q.pose.orientation.w - q.pose.orientation.z * q.pose.orientation.x));
        *yaw   = atan2(2.0 * (q.pose.orientation.z * q.pose.orientation.w + q.pose.orientation.x * q.pose.orientation.y) , - 1.0 + 2.0 * (q.pose.orientation.w * q.pose.orientation.w + q.pose.orientation.x * q.pose.orientation.x));    
    }
}

