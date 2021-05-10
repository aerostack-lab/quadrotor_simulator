#include "AscTecPelicanType/pelican_like_model.h"

using namespace CVG;

PelicanLikeModel::PelicanLikeModel(int idDrone, const std::string &stackPath_in, const std::string &drone_config_filename) :
    Statek(  QUADROTOR_MODEL_STATE_LENGTH ),
    Statek1( QUADROTOR_MODEL_STATE_LENGTH ),
    Inputs(  QUADROTOR_MODEL_INPUT_LENGTH ),
    Observation( QUADROTOR_MODEL_OBSERVATION_LENGTH )
{
    timer.restart(false);

    std::cout << "Constructor: PelicanLikeModel" << std::endl;
    try {
        //XMLFileReader my_xml_reader( stackPath_in+"quadrotor_simulator/configs/"+drone_config_filename);
        //JL FIX
        XMLFileReader my_xml_reader( drone_config_filename);

        DYAW_SCALE = my_xml_reader.readDoubleValue( "pelican_like_model:yaw_dynamics:DYAW_SCALE" );
        Tp_Y       = my_xml_reader.readDoubleValue( "pelican_like_model:yaw_dynamics:Tp_Y" );
        Zeta_Y     = my_xml_reader.readDoubleValue( "pelican_like_model:yaw_dynamics:Zeta_Y" );

        PITCH_SCALE = my_xml_reader.readDoubleValue( "pelican_like_model:pitch_roll_dynamics:PITCH_SCALE" );
        Tp_P        = my_xml_reader.readDoubleValue( "pelican_like_model:pitch_roll_dynamics:Tp_P" );
        ROLL_SCALE  = my_xml_reader.readDoubleValue( "pelican_like_model:pitch_roll_dynamics:ROLL_SCALE" );
        Tp_R        = my_xml_reader.readDoubleValue( "pelican_like_model:pitch_roll_dynamics:Tp_R" );

        THRUST_SCALE = my_xml_reader.readDoubleValue( "pelican_like_model:vertical_dynamics:THRUST_SCALE" );
        Tth          = my_xml_reader.readDoubleValue( "pelican_like_model:vertical_dynamics:Tth" );
        m            = my_xml_reader.readDoubleValue( "pelican_like_model:vertical_dynamics:m" );
        vzmax        = my_xml_reader.readDoubleValue( "pelican_like_model:vertical_dynamics:vzmax" );

        ki  = my_xml_reader.readDoubleValue( "pelican_like_model:horizontal_dynamics:ki" );
        ci  = my_xml_reader.readDoubleValue( "pelican_like_model:horizontal_dynamics:ci" );
        ktr = my_xml_reader.readDoubleValue( "pelican_like_model:horizontal_dynamics:ktr" );

        g = 9.81;
    } catch ( cvg_XMLFileReader_exception &e) {
        throw cvg_XMLFileReader_exception(std::string("[cvg_XMLFileReader_exception! caller_function: ") + BOOST_CURRENT_FUNCTION + e.what() + "]\n");
    }
}

//Parrot Modelo 1
void PelicanLikeModel::processModel(Vector* Statek1, Vector* Statek, Vector* Inputs)
{
    // Reading statek values

    float P_k = Statek->getValueData(1);
    float R_k = Statek->getValueData(2);
    float Y_k = Statek->getValueData(3);
    float dY_k = Statek->getValueData(4);
    float d2Y_k = Statek->getValueData(5);
    float T_k = Statek->getValueData(6);
    float Z_k = Statek->getValueData(7);
    float dZ_k = Statek->getValueData(8);
    float x_k = Statek->getValueData(9);
    float y_k = Statek->getValueData(10);
    float vx_k = Statek->getValueData(11);
    float vy_k = Statek->getValueData(12);


    // Reading input values

    float Pc_ref = Inputs->getValueData(1);
    float Rc_ref = Inputs->getValueData(2);
    float dYc_ref = Inputs->getValueData(3);
    float Tc_ref = Inputs->getValueData(4);
    float mr_ref = Inputs->getValueData(5);


    // Reference value gains are already included in the State Model


    // Assignment to state in k+1

    Statek1->setValueData(P_k-timeIntegration*(P_k/Tp_P-(Pc_ref*1.745329251994221E-2)/(PITCH_SCALE*Tp_P))*1.0,1);
    Statek1->setValueData(R_k-timeIntegration*(R_k/Tp_R-(Rc_ref*1.745329251994221E-2)/(ROLL_SCALE*Tp_R))*1.0,2);
    Statek1->setValueData(Y_k+dY_k*timeIntegration,3);
    Statek1->setValueData(dY_k+d2Y_k*timeIntegration,4);
    Statek1->setValueData(d2Y_k-timeIntegration*((1.0/(Tp_Y*Tp_Y)*dY_k*-9.0)/(Zeta_Y*Zeta_Y-1.0)+(1.0/(Tp_Y*Tp_Y)*dYc_ref*1.570796326794834E-1)/(DYAW_SCALE*(Zeta_Y*Zeta_Y-1.0))+(Zeta_Y*d2Y_k*1.0/sqrt((Zeta_Y*Zeta_Y)*-1.0+1.0)*6.0)/Tp_Y)*1.0,5);
    Statek1->setValueData(T_k-timeIntegration*(dZ_k*-1.0+T_k/Tth+Tc_ref/Tth)*1.0,6);
    Statek1->setValueData(Z_k+dZ_k*timeIntegration,7);
    Statek1->setValueData(dZ_k+timeIntegration*(g*mr_ref+(dZ_k*(g*m-3.2E1))/(m*vzmax)+(T_k*cos(P_k)*cos(R_k))/(THRUST_SCALE*m)),8);
    Statek1->setValueData(x_k+timeIntegration*vx_k,9);
    Statek1->setValueData(y_k+timeIntegration*vy_k,10);
    Statek1->setValueData(vx_k-ktr*timeIntegration*(g*cos(Y_k)*sin(P_k)+g*sin(R_k)*sin(Y_k)+ki*vx_k*(ci+sqrt(vx_k*vx_k+vy_k*vy_k)))*1.0,11);
    Statek1->setValueData(vy_k-ktr*timeIntegration*(g*cos(Y_k)*sin(R_k)*-1.0+g*sin(P_k)*sin(Y_k)+ki*vy_k*(ci+sqrt(vx_k*vx_k+vy_k*vy_k)))*1.0,12);

//    T_k = Statek->getValueData(6);
//    Z_k = Statek->getValueData(7);
//    dZ_k = Statek->getValueData(8);
//    std::cout << "Tc_ref:" << Tc_ref << " T_k:" << T_k << " Z_k:" << Z_k << " dZ_k:" << dZ_k << std::endl;

    return;
}


//User Observation model
void PelicanLikeModel::observationModel(Vector* Output, Vector* Statek)
{
    // Reading statek values

    float P_k = Statek->getValueData(1);
    float R_k = Statek->getValueData(2);
    float Y_k = Statek->getValueData(3);
    float dY_k = Statek->getValueData(4);
//    float d2Y_k = Statek->getValueData(5);
//    float T_k = Statek->getValueData(6);
    float Z_k = Statek->getValueData(7);
    float dZ_k = Statek->getValueData(8);
    float x_k = Statek->getValueData(9);
    float y_k = Statek->getValueData(10);
    float vx_k = Statek->getValueData(11);
    float vy_k = Statek->getValueData(12);


    // Filling in Output/measurements vector

    Output->setValueData(P_k,1);
    Output->setValueData(R_k,2);
    Output->setValueData(Y_k,3);
    Output->setValueData(dY_k,4);
    Output->setValueData(Z_k,5);
    Output->setValueData(dZ_k,6);
    Output->setValueData(x_k,7);
    Output->setValueData(y_k,8);
    Output->setValueData(vx_k,9);
    Output->setValueData(vy_k,10);
    Output->setValueData(vx_k*cos(Y_k) + vy_k*sin(Y_k),11);
    Output->setValueData(vy_k*cos(Y_k) - 1.0*vx_k*sin(Y_k),12);

	return;
}


//User Jacobian process model
void PelicanLikeModel::jacobiansProcessModel(Matrix* MatJacFx, Matrix* MatJacFu, Vector* Statek, Vector* Inputs)
{
    const double eps_jpp = 1e-6;
    // Reading statek values

    float P_k = Statek->getValueData(1);
    float R_k = Statek->getValueData(2);
    float Y_k = Statek->getValueData(3);
//    float dY_k = Statek->getValueData(4);
//    float d2Y_k = Statek->getValueData(5);
    float T_k = Statek->getValueData(6);
//    float Z_k = Statek->getValueData(7);
//    float dZ_k = Statek->getValueData(8);
//    float x_k = Statek->getValueData(9);
//    float y_k = Statek->getValueData(10);
    float vx_k = Statek->getValueData(11);
    float vy_k = Statek->getValueData(12);


    // Filling in Process Jacobian, MatJacFx

    // check sqrt(vx_k*vx_k+vy_k*vy_k) ~= 0
    if (sqrt(vx_k*vx_k+vy_k*vy_k) < eps_jpp) {
        vx_k = 0.0;
        vy_k = 0.0;
    }

    MatJacFx->setValueData((timeIntegration*-1.0)/Tp_P+1.0,1,1);
    MatJacFx->setValueData((timeIntegration*-1.0)/Tp_R+1.0,2,2);
    MatJacFx->setValueData(1.0,3,3);
    MatJacFx->setValueData(timeIntegration,3,4);
    MatJacFx->setValueData(1.0,4,4);
    MatJacFx->setValueData(timeIntegration,4,5);
    MatJacFx->setValueData((1.0/(Tp_Y*Tp_Y)*timeIntegration*9.0)/(Zeta_Y*Zeta_Y-1.0),5,4);
    MatJacFx->setValueData((Zeta_Y*timeIntegration*1.0/sqrt((Zeta_Y*Zeta_Y)*-1.0+1.0)*-6.0)/Tp_Y+1.0,5,5);
    MatJacFx->setValueData((timeIntegration*-1.0)/Tth+1.0,6,6);
    MatJacFx->setValueData(timeIntegration,6,8);
    MatJacFx->setValueData(1.0,7,7);
    MatJacFx->setValueData(timeIntegration,7,8);
    MatJacFx->setValueData((T_k*timeIntegration*cos(R_k)*sin(P_k)*-1.0)/(THRUST_SCALE*m),8,1);
    MatJacFx->setValueData((T_k*timeIntegration*cos(P_k)*sin(R_k)*-1.0)/(THRUST_SCALE*m),8,2);
    MatJacFx->setValueData((timeIntegration*cos(P_k)*cos(R_k))/(THRUST_SCALE*m),8,6);
    MatJacFx->setValueData((timeIntegration*(g*m-3.2E1))/(m*vzmax)+1.0,8,8);
    MatJacFx->setValueData(1.0,9,9);
    MatJacFx->setValueData(timeIntegration,9,11);
    MatJacFx->setValueData(1.0,10,10);
    MatJacFx->setValueData(timeIntegration,10,12);
    MatJacFx->setValueData(g*ktr*timeIntegration*cos(P_k)*cos(Y_k)*-1.0,11,1);
    MatJacFx->setValueData(g*ktr*timeIntegration*cos(R_k)*sin(Y_k)*-1.0,11,2);
    MatJacFx->setValueData(ktr*timeIntegration*(g*cos(Y_k)*sin(R_k)-g*sin(P_k)*sin(Y_k)*1.0)*-1.0,11,3);
    MatJacFx->setValueData(ktr*timeIntegration*(ki*(ci+sqrt(vx_k*vx_k+vy_k*vy_k))+ki*(vx_k*vx_k)*1.0/sqrt(vx_k*vx_k+vy_k*vy_k))*-1.0+1.0,11,11);
    MatJacFx->setValueData(ki*ktr*timeIntegration*vx_k*vy_k*1.0/sqrt(vx_k*vx_k+vy_k*vy_k)*-1.0,11,12);
    MatJacFx->setValueData(g*ktr*timeIntegration*cos(P_k)*sin(Y_k)*-1.0,12,1);
    MatJacFx->setValueData(g*ktr*timeIntegration*cos(R_k)*cos(Y_k),12,2);
    MatJacFx->setValueData(ktr*timeIntegration*(g*cos(Y_k)*sin(P_k)+g*sin(R_k)*sin(Y_k))*-1.0,12,3);
    MatJacFx->setValueData(ki*ktr*timeIntegration*vx_k*vy_k*1.0/sqrt(vx_k*vx_k+vy_k*vy_k)*-1.0,12,11);
    MatJacFx->setValueData(ktr*timeIntegration*(ki*(ci+sqrt(vx_k*vx_k+vy_k*vy_k))+ki*(vy_k*vy_k)*1.0/sqrt(vx_k*vx_k+vy_k*vy_k))*-1.0+1.0,12,12);


    // Filling in Process Jacobian, MatJacFu

    MatJacFu->setValueData((timeIntegration*1.745329251994221E-2)/(PITCH_SCALE*Tp_P),1,1);
    MatJacFu->setValueData((timeIntegration*1.745329251994221E-2)/(ROLL_SCALE*Tp_R),2,2);
    MatJacFu->setValueData((1.0/(Tp_Y*Tp_Y)*timeIntegration*-1.570796326794834E-1)/(DYAW_SCALE*(Zeta_Y*Zeta_Y-1.0)),5,3);
    MatJacFu->setValueData((timeIntegration*-1.0)/Tth,6,4);
    MatJacFu->setValueData(g*timeIntegration,8,5);

	return;
}


//User Jacobian of observation model
void PelicanLikeModel::jacobiansObservationModel(Matrix* MatJacHx, Vector* Statek)
{
    // Reading statek values

//    float P_k = Statek->getValueData(1);
//    float R_k = Statek->getValueData(2);
    float Y_k = Statek->getValueData(3);
//    float dY_k = Statek->getValueData(4);
//    float d2Y_k = Statek->getValueData(5);
//    float T_k = Statek->getValueData(6);
//    float Z_k = Statek->getValueData(7);
//    float dZ_k = Statek->getValueData(8);
//    float x_k = Statek->getValueData(9);
//    float y_k = Statek->getValueData(10);
    float vx_k = Statek->getValueData(11);
    float vy_k = Statek->getValueData(12);


    // Filling in Observation Jacobian

    MatJacHx->setValueData(1.0,1,1);
    MatJacHx->setValueData(1.0,2,2);
    MatJacHx->setValueData(1.0,3,3);
    MatJacHx->setValueData(1.0,4,4);
    MatJacHx->setValueData(1.0,5,7);
    MatJacHx->setValueData(1.0,6,8);
    MatJacHx->setValueData(1.0,7,9);
    MatJacHx->setValueData(1.0,8,10);
    MatJacHx->setValueData(1.0,9,11);
    MatJacHx->setValueData(1.0,10,12);
    MatJacHx->setValueData(vy_k*cos(Y_k) - 1.0*vx_k*sin(Y_k),11,3);
    MatJacHx->setValueData(cos(Y_k),11,11);
    MatJacHx->setValueData(sin(Y_k),11,12);
    MatJacHx->setValueData(- 1.0*vx_k*cos(Y_k) - 1.0*vy_k*sin(Y_k),12,3);
    MatJacHx->setValueData(-1.0*sin(Y_k),12,11);
    MatJacHx->setValueData(cos(Y_k),12,12);

    return;
}

void PelicanLikeModel::setInputs(double pitch_in, double roll_in, double dyaw_in, double thrust_in)
{
    Inputs.setValueData( pitch_in, 1);
    Inputs.setValueData(  roll_in, 2);
    Inputs.setValueData(  dyaw_in, 3);
    Inputs.setValueData(thrust_in, 4);
    Inputs.setValueData(      1.0, 5);  // mr = 1.0 approx.
}

void PelicanLikeModel::getObservation(double &x_out, double &y_out, double &z_out, double &yaw_out, double &pitch_out, double &roll_out, double &vx_out, double &vy_out, double &vz_out, double &dyaw_out, double &vxm_out, double &vym_out)
{
    observationModel( &Observation, &Statek);

    pitch_out = Observation.getValueData(1);
    roll_out  = Observation.getValueData(2);
    yaw_out   = Observation.getValueData(3);
    dyaw_out  = Observation.getValueData(4);
    yaw_out   = atan2( sin(yaw_out), cos(yaw_out));
    Statek.setValueData(         yaw_out,3);
    z_out     = Observation.getValueData(5);
    vz_out    = Observation.getValueData(6);
    x_out     = Observation.getValueData(7);
    y_out     = Observation.getValueData(8);
    vx_out    = Observation.getValueData(9);
    vy_out    = Observation.getValueData(10);
    vxm_out   = Observation.getValueData(11);
    vym_out   = Observation.getValueData(12);
}

void PelicanLikeModel::runSimulation()
{
    double elapsed_seconds = timer.getElapsedSeconds();
    timer.restart(true);

    timeIntegration = elapsed_seconds;
    processModel( &Statek1, &Statek, &Inputs);

    Statek = Statek1;
    return;
}

void PelicanLikeModel::start(double x_in, double y_in, double z_in, double yaw_in, double pitch_in, double roll_in, double vx_in, double vy_in, double vz_in, double dyaw_in)
{
    timer.restart(false);
    setState( x_in,  y_in,  z_in,  yaw_in,  pitch_in,  roll_in,  vx_in,  vy_in,  vz_in,  dyaw_in);
}

void PelicanLikeModel::setState(double x_in, double y_in, double z_in, double yaw_in, double pitch_in, double roll_in, double vx_in, double vy_in, double vz_in, double dyaw_in)
{
    Statek.setValueData( pitch_in, 1);
    Statek.setValueData(  roll_in, 2);
    Statek.setValueData(   yaw_in, 3);
    Statek.setValueData(  dyaw_in, 4);
//    Statek.setValueData(      0.0, 5); // Thrust
    Statek.setValueData(     z_in, 7);
    Statek.setValueData(    vz_in, 8);
    Statek.setValueData(     x_in, 9);
    Statek.setValueData(     y_in,10);
    Statek.setValueData(    vx_in,11);
    Statek.setValueData(    vy_in,12);

    Statek1 = Statek;
}
