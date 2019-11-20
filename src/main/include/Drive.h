#pragma once

#include "drive/SparkMaxEnclosure.h"
#include "drive/SwerveEnclosure.h"

#include "IOMap.h"

#include <memory>

class Drive {
public:
    Drive();
    SparkMaxEnclosure module{"swerve 1", CAN_SWERVE_FRONT_LEFT_MOVE, CAN_SWERVE_FRONT_LEFT_TURN};
private:
    
    


};