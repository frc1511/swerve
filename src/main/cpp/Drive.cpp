#include "Drive.h"

// Drive::Drive() : module("module", moduleDriver, moduleTurn, 9/1) {
Drive::Drive(){

    // printf("%ld, %ld\n", moduleDriver, moduleTurn);
    // // moduleDriver = std::make_shared<rev::SparkMax>(3);
    // // moduleTurn = std::make_shared<WPI_TalonSRX>(7);

    // printf("%ld, %ld", moduleDriver, moduleTurn);   

    swerve.setSpeedLimiter(.35);

    gyro.Reset();

    printf("Constructing drive\n");
    
}