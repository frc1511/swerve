#include "Controls.h"

Controls::Controls(Drive* drive) {
    this->drive = drive;
    timer.Reset();
    timer.Start();
}

void Controls::process() {
    // double angle = joystick.GetDirectionDegrees();
    
    double angle = joystick.GetDirectionDegrees()/360;
    // if(joystick.GetPOV() != -1){
    //     angle = joystick.GetPOV()/360;
    // }

    double moveMent = .25;//joystick.GetRawAxis(3) - joystick.GetRawAxis(2);
    
    // drive->module0.MoveWheel(rotation   , angle);
    drive->module1.MoveWheel( moveMent   , angle);
    // drive->module2.MoveWheel(rotation   , angle);
    // drive->module3.MoveWheel(rotation   , angle);
    double forward = joystick.GetRawAxis(1);
    if(abs(forward) < .1) forward = 0;

    double strafe = joystick.GetRawAxis(0);
    if(abs(strafe) < .1) strafe = 0;
    

    // drive->swerve.move(0, strafe, 0,0);

    printf("angle = %f  ", angle);
}