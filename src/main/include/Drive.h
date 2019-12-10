#pragma once

#include "drive/SparkMaxEnclosure.h"
#include "drive/SwerveEnclosure.h"
#include "Drive/RobotDriveSwerve.h"

#include "IOMap.h"

#include <memory>

class Drive {
public:
    Drive();
    SparkTalonEnclosure module0{"front left", CAN_SWERVE_FRONT_LEFT_MOVE, CAN_SWERVE_FRONT_LEFT_TURN};
    SparkTalonEnclosure module1{"front right", CAN_SWERVE_FRONT_RIGHT_MOVE, CAN_SWERVE_FRONT_RIGHT_TURN};
    SparkTalonEnclosure module2{"rear left", CAN_SWERVE_REAR_LEFT_MOVE, CAN_SWERVE_REAR_LEFT_TURN};
    SparkTalonEnclosure module3{"rear right", CAN_SWERVE_REAR_RIGHT_MOVE, CAN_SWERVE_REAR_RIGHT_TURN};

    // RobotDriveSwerve swerve{&module0, &module1, &module2, &module3};
    
private:
    
    


};