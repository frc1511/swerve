#pragma once

#include "frc/WPILib.h"
#include "Drive.h"

class Controls {
public:
    Controls(Drive* drive);
    void process();
private:
    frc::Joystick joystick{0};
    double angle = 0;

    Drive* drive;
    frc::Timer timer;

};