#ifndef PELICAN_LIKE_ELEMENTS_H
#define PELICAN_LIKE_ELEMENTS_H

//I/O stream
//std::cout
#include <iostream>

//Math
#include <cmath>

//White noise (stdc++11)
//#include <random>
#include "cvg_random.h"

#include "ARDroneType/droneElements.h"

/////////////////////////////////////////
// Class Battery
//
//   Description
//
/////////////////////////////////////////
class PL_BatterySensor : public Battery
{};

/////////////////////////////////////////
// Class RotationAnglesSensor
//
//   Description
//
/////////////////////////////////////////
class PL_RotationAnglesSensor : public RotationAnglesSensor
{};

/////////////////////////////////////////
// Class AltitudeSensor
//
//   Description
//
/////////////////////////////////////////
class PL_AltitudeSensor : public AltitudeSensor
{};

/////////////////////////////////////////
// Class GroundSpeedSensor
//
//   Description
//
/////////////////////////////////////////
class PL_GroundSpeedSensor : public GroundSpeedSensor
{};

/////////////////////////////////////////
// Class Autopilot
//
//   Description
//
/////////////////////////////////////////
class PL_LLCommandReceiver {
    //Commands
protected:
    double pitch_command;
    double roll_command;
    double dyaw_command;
    double thrust_command;
    double mass_disturbance_command;

public:
    PL_LLCommandReceiver();
    ~PL_LLCommandReceiver();

public:
    //Pitch & roll
    int setCommand(double pitch_command_in,   double roll_command_in,   double dyaw_command_in,   double thrust_command_in);
    int getCommand(double &pitch_command_out, double &roll_command_out, double &dyaw_command_out, double &thrust_command_out);
};

#endif // PELICAN_LIKE_ELEMENTS_H
