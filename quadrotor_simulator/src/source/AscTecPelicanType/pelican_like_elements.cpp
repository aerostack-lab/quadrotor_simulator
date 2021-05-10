#include "AscTecPelicanType/pelican_like_elements.h"


PL_LLCommandReceiver::PL_LLCommandReceiver()
{
    pitch_command  = 0.0;
    roll_command   = 0.0;
    dyaw_command   = 0.0;
    thrust_command = 0.0;
    mass_disturbance_command = 1.0;
}

PL_LLCommandReceiver::~PL_LLCommandReceiver()
{
}

int PL_LLCommandReceiver::setCommand(double pitch_command_in, double roll_command_in, double dyaw_command_in, double thrust_command_in)
{
    pitch_command  = pitch_command_in;
    roll_command   = roll_command_in;
    dyaw_command   = dyaw_command_in;
    thrust_command = thrust_command_in;
    return 1;
}

int PL_LLCommandReceiver::getCommand(double &pitch_command_out, double &roll_command_out, double &dyaw_command_out, double &thrust_command_out)
{
    pitch_command_out  = pitch_command;
    roll_command_out   = roll_command;
    dyaw_command_out   = dyaw_command;
    thrust_command_out = thrust_command;
    return 1;
}
