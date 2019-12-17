#include "drive/RobotDriveSwerve.h"

const double kWidth = 14.0;
const double kLength = 28.0;
RobotDriveSwerve::RobotDriveSwerve(SparkTalonEnclosure* frontLeftWheel,
								   SparkTalonEnclosure* frontRightWheel,
								   SparkTalonEnclosure* rearLeftWheel,
								   SparkTalonEnclosure* rearRightWheel):
								   mathSystem(kLength, kWidth)

{
	this->frontRightWheel = frontRightWheel;
	this->frontLeftWheel = frontLeftWheel;
	this->rearLeftWheel = rearLeftWheel;
	this->rearRightWheel = rearRightWheel;

	if(kLength == 0.0 || kWidth == 0.0)
		throw std::invalid_argument("Swerve drive Width/Length cannot be zero");
}

void RobotDriveSwerve::move(double x, double y, double rotation, double angle)
{
	double** wheelValues;
	x *= -1;

	if(GetMode() == kFieldCentric) {
		wheelValues = mathSystem.Calculate(x, y, rotation, angle);
	}
	else {
		wheelValues = mathSystem.Calculate(x, y, rotation);
	}

	frontLeftWheel->MoveWheel(	wheelValues[0][0], wheelValues[0][1], true);
	frontRightWheel->MoveWheel(	wheelValues[1][0], wheelValues[1][1], true);
	rearLeftWheel->MoveWheel(	wheelValues[2][0], wheelValues[2][1], true);
	rearRightWheel->MoveWheel(	wheelValues[3][0], wheelValues[3][1], true);


}

void RobotDriveSwerve::StopMotor()
{
	frontLeftWheel->StopWheel();
	frontRightWheel->StopWheel();
	rearLeftWheel->StopWheel();
	rearRightWheel->StopWheel();
}

int RobotDriveSwerve::GetMode()
{
	return m_mode;
}
void RobotDriveSwerve::SetMode(DriveMode mode)
{
	switch(mode)
	{
	case(kFieldCentric):
		m_mode = kFieldCentric;
		break;
	case(kRobotCentric):
		m_mode = kRobotCentric;
		break;
	default:
		frc::DriverStation::ReportError("ERROR: Invalid drivetrain mode input");
	}
}
void RobotDriveSwerve::ToggleMode() {
	if(GetMode() == kRobotCentric) {
		m_mode = kFieldCentric;
	} else {
		m_mode = kRobotCentric;
	}
}
