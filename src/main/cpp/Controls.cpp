#include "Controls.h"

Controls::Controls(Drive* drive) {
    this->drive = drive;
    timer.Reset();
    timer.Start();
}

void Controls::process() {
    // double angle = joystick.GetDirectionDegrees();
    
    // double angle = joystick.GetDirectionDegrees();
    // if(joystick.GetPOV() != -1){
    //     angle = joystick.GetPOV();
    // }

    // double rotation = joystick.GetRawAxis(3) - joystick.GetRawAxis(2);
    
    // drive->module0.MoveWheel(rotation   , angle);
    // drive->module1.MoveWheel(rotation   , angle);
    // drive->module2.MoveWheel(rotation   , angle);
    // drive->module3.MoveWheel(rotation   , angle);
    drive->swerve.move(joystick.GetRawAxis(1)/2, joystick.GetRawAxis(0)/2, 0);//rotation/2);

    // printf("angle = %f\n", angle);
}