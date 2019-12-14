#include "Controls.h"

Controls::Controls(Drive* drive) {
    this->drive = drive;
    timer.Reset();
    timer.Start();
}

void Controls::process() {
    /* We Need a Bring Up process to confirm modules are setup right and our constants are correct */

    /* STEP1: read and confirm, module by module, the encoders are scaled correctly, hand turn */
    double currentAngle = drive->module0.GetRotationalPos();
    // double currentAngle = drive->module1.GetRotationalPos();
    // double currentAngle = drive->module2.GetRotationalPos();
    // double currentAngle = drive->module3.GetRotationalPos();
    // printf("currentAngle = %f\n", currentAngle);
    
    /* STEP2: confirm PID values are right, start by having a set value, and enabling, then move to joystick 
            why are we  overshooting and occilating*/
    // if(joystick.GetPOV() != -1) angle = joystick.GetPOV()/360.0;
    angle = .25;
    double move = joystick.GetRawAxis(3);
    // if(joystick.GetMagnitude() > 0.3) angle = joystick.GetDirectionDegrees()/360;
    drive->module0.MoveWheel( move  , angle, false);
    printf("requestedAngle = %f encoderValue = %f\n",angle, drive->module0.GetRawEncoderVal());

    /* STEP3: confirm all module will do the same thing */
    // drive->module1.MoveWheel( move  , angle, false);
    // drive->module2.MoveWheel( move  , angle, false);
    // drive->module3.MoveWheel( move  , angle, false);


    // double rotation = joystick.GetRawAxis(3) - joystick.GetRawAxis(2);
    // drive->swerve.move(joystick.GetRawAxis(1), joystick.GetRawAxis(0), rotation);
    
    
    
}