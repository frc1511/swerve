#include "Controls.h"

Controls::Controls(Drive* drive) {
    this->drive = drive;
    timer.Reset();
    timer.Start();
}

void Controls::process() {
    double angle = joystick.GetDirectionDegrees()+180;
    // double angle = joystick.GetDirectionDegrees();

    if(timer.Get()*100 > 360) timer.Reset();
    
    angle = timer.Get()*100;

    drive->module.MoveWheel(.05   , angle);
    // drive->module.SetAngle(angle);
    // printf("%d, %f\n", drive->module.GetEncoderVal(), angle);
    printf("%f\n", angle);
}