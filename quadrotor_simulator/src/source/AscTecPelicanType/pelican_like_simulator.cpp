#include "AscTecPelicanType/pelican_like_simulator.h"

PelicanLikeSimulator::PelicanLikeSimulator(int idDrone, const std::string &stackPath_in, const std::string &drone_config_filename) :
    pelican_like_model(idDrone, stackPath_in, drone_config_filename)
{
    // Read initial position from configuration ekf_state_estimator_config file
    try {
        XMLFileReader my_xml_reader(stackPath_in+"configs/drone"+cvg_int_to_string(idDrone)+"/ekf_state_estimator_config.xml");
        double x_aux, y_aux, z_aux, yaw_aux, pitch_aux, roll_aux;
        x_aux = my_xml_reader.readDoubleValue( "take_off_site:position:x" );
        y_aux = my_xml_reader.readDoubleValue( "take_off_site:position:y" );
        z_aux = my_xml_reader.readDoubleValue( "take_off_site:position:z" );
        yaw_aux   = (M_PI/180.0)*my_xml_reader.readDoubleValue( "take_off_site:attitude:yaw" );
        pitch_aux = (M_PI/180.0)*my_xml_reader.readDoubleValue( "take_off_site:attitude:pitch" );
        roll_aux  = (M_PI/180.0)*my_xml_reader.readDoubleValue( "take_off_site:attitude:roll" );
        setPosition_drone_GMR_wrt_GFF( x_aux, y_aux, z_aux, yaw_aux, pitch_aux, roll_aux);
        setSpeed_drone_GMR_wrt_GFF( 0.0, 0.0, 0.0);
        pelican_like_model.start( current_xl, current_yl, current_zl, current_yawl, current_pitchl, current_rolll, current_vxl, current_vyl, current_vzl, current_dyawl);
    } catch ( cvg_XMLFileReader_exception &e) {
        throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
    }
}

PelicanLikeSimulator::~PelicanLikeSimulator()
{
}

int PelicanLikeSimulator::run()
{
    // Run the model/simulation
    double pitch_counts, roll_counts, dyaw_counts, thrust_counts;
    LLcommandReceiver.getCommand( pitch_counts, roll_counts, dyaw_counts, thrust_counts);

    // values of pose before running this simulation step
    // drone_LMrT_wrt_LMrTFF, l ~ local
    double current_xl_km1, current_yl_km1, current_zl_km1, current_yawl_km1, current_pitchl_km1, current_rolll_km1;
    double current_vxl_km1, current_vyl_km1, current_vzl_km1, current_dyawl_km1;
    // local_groundspeed
    double current_vxm_km1, current_vym_km1;

    pelican_like_model.getObservation( current_xl_km1, current_yl_km1, current_zl_km1,
                                        current_yawl_km1, current_pitchl_km1, current_rolll_km1,
                                        current_vxl_km1,  current_vyl_km1, current_vzl_km1,
                                        current_dyawl_km1,
                                        current_vxm_km1, current_vym_km1);

    pelican_like_model.setInputs( pitch_counts, roll_counts, dyaw_counts, thrust_counts);
    pelican_like_model.runSimulation();

    pelican_like_model.getObservation( current_xl, current_yl, current_zl,
                                        current_yawl, current_pitchl, current_rolll,
                                        current_vxl,  current_vyl, current_vzl,
                                        current_dyawl,
                                        current_vxm, current_vym);
    if ( (-current_zl) < altitude_threshold_m ) {
        // altitude to low >> no yaw, pitch or roll rotation; no horizontal movement.
        // altitude to low >> only vertical movement allowed.
        current_xl = current_xl_km1;
        current_yl = current_yl_km1;
        current_yawl = current_yawl_km1;
        current_pitchl = 0.0;
        current_rolll  = 0.0;
        current_vxl = 0.0;
        current_vyl = 0.0;
        current_dyawl = 0.0;
        current_vxm = 0.0;
        current_vym = 0.0;
        if ( (-current_zl) < 0.0 ) {
            current_zl  = 0.0;
            current_vzl = 0.0f;
        }
        pelican_like_model.setState( current_xl, current_yl, current_zl, current_yawl, current_pitchl, current_rolll, current_vxl, current_vyl, current_vzl, current_dyawl);
    }

    setPosition_drone_LMrT_wrt_LMrTFF( current_xl, current_yl, current_zl,
                                       current_yawl, current_pitchl, current_rolll);
    setSpeed_droneLMrT_wrt_LMrTFF( current_vxl,  current_vyl, current_vzl);

    // Setting sensors outputs.
    batterySensor.setPercenaje(99.9);
    rotationAnglesSensor.setRotationAngles( current_yawl  *(180.0/M_PI),
                                            current_pitchl*(180.0/M_PI),
                                            current_rolll *(180.0/M_PI));
    groundSpeedSensor.setGroundSpeed( current_vxm,
                                      current_vym);
    altitudeSensor.setAltitude(      current_zl,  0.01*0.01);
    altitudeSensor.setAltitudeSpeed( current_vzl, 0.01*0.01);

    return 1;
}

void PelicanLikeSimulator::setPosition_drone_GMR_wrt_GFF(double x_in, double y_in, double z_in, double yaw_in, double pitch_in, double roll_in)
{
    current_xg = x_in;     current_yg = y_in;         current_zg = z_in;
    current_yawg = yaw_in; current_pitchg = pitch_in; current_rollg = roll_in;

    current_xl = x_in;      current_yl = -y_in;         current_zl = -z_in;
    current_yawl = -yaw_in; current_pitchl = -pitch_in; current_rolll = roll_in;
}

void PelicanLikeSimulator::setPosition_drone_LMrT_wrt_LMrTFF(double x_in, double y_in, double z_in, double yaw_in, double pitch_in, double roll_in)
{
    current_xg = x_in;      current_yg = -y_in;         current_zg = -z_in;
    current_yawg = -yaw_in; current_pitchg = -pitch_in; current_rollg = roll_in;

    current_xl = x_in;     current_yl = y_in;         current_zl = z_in;
    current_yawl = yaw_in; current_pitchl = pitch_in; current_rolll = roll_in;
}

void PelicanLikeSimulator::setSpeed_drone_GMR_wrt_GFF(double vx_in, double vy_in, double vz_in)
{
    current_vxg = vx_in; current_vyg = vy_in;  current_vzg = vz_in;
    current_vxl = vx_in; current_vyl = -vy_in; current_vzl = -vz_in;
}

void PelicanLikeSimulator::setSpeed_droneLMrT_wrt_LMrTFF(double vx_in, double vy_in, double vz_in)
{
    current_vxg = vx_in; current_vyg = -vy_in; current_vzg = -vz_in;
    current_vxl = vx_in; current_vyl = vy_in;  current_vzl = vz_in;
}

void PelicanLikeSimulator::getPosition_drone_GMR_wrt_GFF(double &x_in, double &y_in, double &z_in, double &yaw_in, double &pitch_in, double &roll_in)
{
    x_in = current_xg;     y_in = current_yg;         z_in = current_zg;
    yaw_in = current_yawg; pitch_in = current_pitchg; roll_in = current_rollg;
}

void PelicanLikeSimulator::getPosition_drone_LMrT_wrt_LMrTFF(double &x_in, double &y_in, double &z_in, double &yaw_in, double &pitch_in, double &roll_in)
{
    x_in = current_xl;     y_in = current_yl;         z_in = current_zl;
    yaw_in = current_yawl; pitch_in = current_pitchl; roll_in = current_rolll;
}

void PelicanLikeSimulator::getSpeed_drone_GMR_wrt_GFF(double &vx_in, double &vy_in, double &vz_in)
{
    vx_in = current_vxg;     vy_in = current_vyg;         vz_in = current_vzg;
}

void PelicanLikeSimulator::getSpeed_droneLMrT_wrt_LMrTFF(double &vx_in, double &vy_in, double &vz_in)
{
    vx_in = current_vxl;     vy_in = current_vyl;         vz_in = current_vzl;
}
