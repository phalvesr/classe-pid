#include <iostream>
#include <thread>

#include "PID.h"

int main()
{

    PID::PidParameters parameters = PID::PidParameters();
    parameters.proportionalGain = 0.0010;
    parameters.integralGain = 0.05;
    parameters.derivativeGain = 2.5;
    parameters.samplingTimeInSeconds = 0.3;

    PID::PID pid = PID::PID(parameters);
    pid.setSetPoint(500);

    
    while (true)
    {
        pid.executePiAlgorith();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
}


