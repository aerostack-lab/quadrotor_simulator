#ifndef QUADROTOR_SIMULATOR_H
#define QUADROTOR_SIMULATOR_H


//// ROS  ///////
#include <iostream>
#include "ros/ros.h"
#include <robot_process.h>
#include <cmath>

//Sensor
#include "sensor_msgs/FluidPressure.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/BatteryState.h"

//Geometry
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"

//Aerostack
#include "aerostack_msgs/FlightActionCommand.h"
#include "aerostack_msgs/FlightState.h"

#include "drone_utils/drone_state_command_enum.h"
#include "drone_utils/drone_state_enum.h"
#include <tf/transform_datatypes.h>

#include "ARDroneType/quadrotor_simulator.h"

class QuadrotorSimulatorProcess : public RobotProcess
{
public:
    QuadrotorSimulatorProcess();
    ~QuadrotorSimulatorProcess();
    double get_moduleRate();

private: /*RobotProcess*/
    bool resetValues();
    void ownSetUp();
    void ownStart();
    void ownStop();
    void ownRun();
    double rate;
    std::string robot_namespace;
    std::string robot_config_path;
    std::string configFile;

    std::string altitude_topic;
    std::string sensor_speed_topic;
    std::string battery_state_topic;
    std::string magnetometer_topic;
    std::string pressure_topic;
    std::string temperature_topic;
    std::string estimated_speed_topic;
    std::string estimated_pose_topic;
    std::string roll_pitch_topic;
    std::string altitude_yaw_rate_topic;
    std::string flight_action_topic;
    std::string flight_state_topic;

private:
    QuadrotorSimulator quadrotor_simulator;
    ros::Time last_speed_time;
    double pose_roll, pose_pitch, pose_yaw;
    double last_roll, last_pitch, last_yaw;

    //Subscribers
    ros::Subscriber roll_pitch_sub;
    void rollPitchCallback(const geometry_msgs::PoseStamped& msg);
    ros::Subscriber altitude_rate_yaw_rate_sub;
    void altitudeRateYawRateCallback(const geometry_msgs::TwistStamped& msg);
    ros::Subscriber flight_action_sub;
    void flightActionCallback(const aerostack_msgs::FlightActionCommand& msg);

    //Publishers
    ros::Publisher estimated_pose_pub;
    geometry_msgs::PoseStamped estimated_pose_msg;

    ros::Publisher estimated_speed_pub;
    geometry_msgs::TwistStamped estimated_speed_msg;

    ros::Publisher altitude_pub;
    geometry_msgs::PointStamped altitude_msg;

    ros::Publisher sensor_speed_pub;
    geometry_msgs::TwistStamped sensor_speed_msg;

    ros::Publisher battery_state_pub;
    sensor_msgs::BatteryState battery_msg;

    ros::Publisher magnetometer_pub;
    geometry_msgs::Vector3Stamped magnetometer_msg;

    ros::Publisher pressure_pub;
    sensor_msgs::FluidPressure pressure_msg;

    ros::Publisher flight_state_pub;
    aerostack_msgs::FlightState flight_state_msg;

    ros::Publisher temperature_pub;
    sensor_msgs::Temperature temperature_msg;

    //Publish function
    void publishValues();
    void publishFlightState();
    void toEulerianAngle(geometry_msgs::PoseStamped q, double *roll, double *pitch, double *yaw);
};
#endif
