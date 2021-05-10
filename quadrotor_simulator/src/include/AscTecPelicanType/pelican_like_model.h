#ifndef _QUADROTOR_MODEL_H
#define _QUADROTOR_MODEL_H

// ************** State Vector **************
//  X1,  P  : pitch
//  X2,  R  : roll
//  X3,  Y  : yaw
//  X4,  dY : d(yaw)/dt
//  X5,  d2Y: d2(yaw)/dt2
//  x,y,z: are specified with regards to a fixed world frame
//  X6,  z  : (-1)*altitude
//  X7,  dz : d(z)/dt
//  X8,  d2z: d2(z)/dt2
//  X9,  x  : x
//  X10, y  : y
//  X11, dx : d(x)/dt
//  X12, dy : d(y)/dt
//
// ************** Input Vector **************
//  U1, Pc  : pitch command
//  U2, Rc  : roll command
//  U3, dYc : d(yaw)/dt command
//  U4, Tc  : thrust command
//  U5, mr  : mass ratio mass_model/mass_param
//
// ************** Observation Vector **************
//  Y1,  P   : pitch
//  Y2,  R   : roll
//  Y3,  Y   : yaw
//  Y4,  dY  : d(yaw)/dt
//  x,y,z: are specified with regards to a fixed world frame
//  Y5,  z   : (-1)*altitude
//  Y6,  dz  : d(z)/dt
//  Y7,  x   : x
//  Y8,  y   : y
//  Y9,  dx  : d(x)/dt
//  Y10, dy  : d(y)/dt
//  xm,ym: are specified with regards to a robot body frame
//  Y11, dxm : [Vxm; : [R_Y R_Y]' * [X11;
//  Y12, dym :  Vym] : [R_Y R_Y]     X12]
//



#include "model.h"
#include "Timer.h"
#include "xmlfilereader.h"

// Integration timestep
// extern float timeIntegration;

// Saturation of model input in pitch, roll, dyaw and dz
//#define LIM_PITCH_REF      0.5
//#define LIM_ROLL_REF       0.5
//#define LIM_DYAW_REF       0.5
//#define LIM_DZ_REF         0.5

#define QUADROTOR_MODEL_STATE_LENGTH 12
#define QUADROTOR_MODEL_OBSERVATION_LENGTH 12
#define QUADROTOR_MODEL_INPUT_LENGTH 5

//First model
class PelicanLikeModel : public ContinuousModel {
public:
	// Default constructor
    PelicanLikeModel(int idDrone, const std::string &stackPath_in, const std::string &drone_config_filename=std::string("pelican_like_model.xml"));
	//User process model
    virtual void processModel(CVG::Vector* Statek1, CVG::Vector* Statek, CVG::Vector* Inputs);
	//User Observation model
    virtual void observationModel(CVG::Vector* Output, CVG::Vector* State);
	//User Jacobian process model
    virtual void jacobiansProcessModel(CVG::Matrix* MatJacFx, CVG::Matrix* MatJacFu, CVG::Vector* State, CVG::Vector* Inputs);
	//User Jacobian process model
    virtual void jacobiansObservationModel(CVG::Matrix* MatJacHx, CVG::Vector* State);
    ~PelicanLikeModel() {}

private:
    Timer timer;
    CVG::Vector Statek;         // Current State
    CVG::Vector Statek1;        // State in k+1
    CVG::Vector Inputs;         // Commands: pitch/roll/dyaw/daltitude
    CVG::Vector Observation;    //

public:
    void setInputs( double pitch_in, double roll_in, double dyaw_in, double thrust_in);
    void getObservation( double &x_out, double &y_out, double &z_out,
                         double &yaw_out, double &pitch_out, double &roll_out,
                         double &vx_out, double &vy_out, double &vz_out, double &dyaw_out,
                         double &vxm_out, double &vym_out);
    void runSimulation();
    void start( double x_in, double y_in, double z_in,
                double yaw_in, double pitch_in, double roll_in,
                double vx_in, double vy_in, double vz_in, double dyaw_in);
    void setState( double x_in, double y_in, double z_in,
                   double yaw_in, double pitch_in, double roll_in,
                   double vx_in, double vy_in, double vz_in, double dyaw_in);
    inline void stop() {}

    // Model configuration parameters
private:
    double DYAW_SCALE, Tp_Y, Zeta_Y;
    double PITCH_SCALE, Tp_P, ROLL_SCALE, Tp_R;
    double THRUST_SCALE, Tth, m, vzmax;
    double g, ki, ci, ktr;
};

#endif // _QUADROTOR_MODEL_H
