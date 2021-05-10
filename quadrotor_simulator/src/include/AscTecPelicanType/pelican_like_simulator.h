#ifndef PELICAN_LIKE_SIMULATOR_H
#define PELICAN_LIKE_SIMULATOR_H

#include "AscTecPelicanType/pelican_like_elements.h"
#include "AscTecPelicanType/pelican_like_model.h"

const double altitude_threshold_m = 0.200; // m

class PelicanLikeSimulator
{
private:
    PelicanLikeModel pelican_like_model;
    //Drone elements
public:
    PL_BatterySensor           batterySensor;
    PL_RotationAnglesSensor    rotationAnglesSensor;
    PL_GroundSpeedSensor       groundSpeedSensor;
    PL_AltitudeSensor          altitudeSensor;
    PL_LLCommandReceiver       LLcommandReceiver;

public:
    PelicanLikeSimulator(int idDrone, const std::string &stackPath_in, const std::string &drone_config_filename=std::string("pelican_like_model.xml"));
    ~PelicanLikeSimulator();

    int run();

private: // Current drone position, and speed
    // drone_GMR_wrt_GFF,     g ~ global
    double current_xg, current_yg, current_zg, current_yawg, current_pitchg, current_rollg;
    double current_vxg, current_vyg, current_vzg, current_dyawg;
    // drone_LMrT_wrt_LMrTFF, l ~ local
    double current_xl, current_yl, current_zl, current_yawl, current_pitchl, current_rolll;
    double current_vxl, current_vyl, current_vzl, current_dyawl;
    // local_groundspeed
    double current_vxm, current_vym;

private:  // set position
    void setPosition_drone_GMR_wrt_GFF(double x_in, double y_in, double z_in, double yaw_in, double pitch_in, double roll_in);
    void setPosition_drone_LMrT_wrt_LMrTFF(double x_in, double y_in, double z_in, double yaw_in, double pitch_in, double roll_in);
    void setSpeed_drone_GMR_wrt_GFF( double vx_in, double vy_in, double vz_in);
    void setSpeed_droneLMrT_wrt_LMrTFF( double vx_in, double vy_in, double vz_in);
public:   // get position
    void getPosition_drone_GMR_wrt_GFF(double &x_in, double &y_in, double &z_in, double &yaw_in, double &pitch_in, double &roll_in);
    void getPosition_drone_LMrT_wrt_LMrTFF(double &x_in, double &y_in, double &z_in, double &yaw_in, double &pitch_in, double &roll_in);
    void getSpeed_drone_GMR_wrt_GFF( double &vx_in, double &vy_in, double &vz_in);
    void getSpeed_droneLMrT_wrt_LMrTFF( double &vx_in, double &vy_in, double &vz_in);
    // set inputs
    inline void setInputs( double pitch_in, double roll_in, double dyaw_in, double thrust_in) {
        LLcommandReceiver.setCommand( pitch_in, roll_in, dyaw_in, thrust_in ); }
};

#endif // PELICAN_LIKE_SIMULATOR_H
