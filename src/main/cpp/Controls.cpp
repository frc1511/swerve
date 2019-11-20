#include "Controls.h"

Controls::Controls(Drive* drive) {
    this->drive = drive;
    timer.Reset();
    timer.Start();
}

void Controls::process() {
    // double angle = joystick.GetDirectionDegrees();
    
    // double angle = joystick.GetDirectionDegrees();

    // drive->module.MoveWheel(.05   , angle);
    double rotation = joystick.GetRawAxis(3) - joystick.GetRawAxis(2);
    drive->swerve.move(joystick.GetRawAxis(1), joystick.GetRawAxis(0), rotation);

    // printf("angle = %f\n", angle);
}