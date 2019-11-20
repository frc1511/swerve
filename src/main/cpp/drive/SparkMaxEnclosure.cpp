#include "drive/SparkMaxEnclosure.h"

SparkMaxEnclosure::SparkMaxEnclosure(std::string name, int moveMotorID, int turnMotorID):
									moveMotor(moveMotorID, rev::CANSparkMax::MotorType::kBrushless), turnMotor(turnMotorID)
{
	this->name = name;

	printf("Spark Max Module Initialized\n");
	turnMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative);
	turnMotor.SetSelectedSensorPosition(0);
	// turnMotor.

	SetPID(1, 0, 0, .1);
	// SetInverted(MotorType::TurnMotor, false);
}
SparkMaxEnclosure::~SparkMaxEnclosure(){ return; }

void SparkMaxEnclosure::MoveWheel(double speedVal, double rotationVal)
{
	rotationVal = ConvertAngle(rotationVal, GetEncoderVal());
	printf("rotationVal: %f\n", rotationVal);

	if(ShouldReverse(rotationVal))
	{
		if(rotationVal < 0)
			rotationVal += 0.5;
		else
			rotationVal -= 0.5;

		speedVal *= -1;
	}

	SetSpeed(speedVal);
	if(speedVal != 0.0)
		SetAngle(rotationVal);

	// printf("moving %f, %f\n", speedVal, rotationVal);
}

void SparkMaxEnclosure::StopWheel()
{
	moveMotor.StopMotor();
	turnMotor.StopMotor();
}

void SparkMaxEnclosure::SetInverted(SparkMaxEnclosure::MotorType type, bool val)
{
	if(type == MotorType::TurnMotor)
	{
		turnMotor.SetInverted(val);
	}
	else if(type == MotorType::MoveMotor)
		moveMotor.SetInverted(val);
}

void SparkMaxEnclosure::SetPID(double P, double I, double D, double F) {//Changed
	turnMotor.Config_kP(0, P, 100);
	turnMotor.Config_kI(0, I, 100);
	turnMotor.Config_kD(0, D, 100);
	turnMotor.Config_kF(0, F, 100);
}

//Outputs encoder values for the corresponding motor
int SparkMaxEnclosure::GetEncoderVal()
{
	if(reverseEncoder)
		return -1*turnMotor.GetSelectedSensorPosition();
	else
		return turnMotor.GetSelectedSensorPosition();
}

void SparkMaxEnclosure::SetSpeed(double speedVal)
{
	moveMotor.Set(speedVal); //change back to speedval
	// printf("set speed: %f on CAN %d\n", speedVal, moveMotor.GetDeviceId());
}

void SparkMaxEnclosure::SetAngle(double desiredAngle)
{
	double output;
	if(reverseSteer)
		output =  -1*desiredAngle*kGearRatio;
	else
		output = desiredAngle*kGearRatio;

	turnMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, output);

	printf("desired angle = %f, current = %d\n", output, turnMotor.GetSelectedSensorPosition());
}

bool SparkMaxEnclosure::ShouldReverse(double wa)
{
	double ea = GetEncoderVal();
	ea /= kGearRatio;
	ea = fmod(ea, 1);

	//Convert the next wheel angle, which is from -.5 to .5, to 0 to 1
	if (wa < 0) wa += 1;

	//Find the difference between the two (not sure if the conversion from (-0.5 to 0.5) to (0 to 1) above is needed)
	//Difference between the two points. May be anything between -1 to 1, but we are looking for a number between -.5 to .5
	double longDifference = fabs(wa - ea);

	//finds shortest distance (0 to 0.5), always positive though (which is what we want)
	double difference = fmin(longDifference, 1.0-longDifference);

	//If the sum is greater than 1/4, then return true (aka it is easier for them to turn around and go backwards than go forward)
	if (difference > 0.25) return true;
	else return false;
}

double SparkMaxEnclosure::ConvertAngle(double angle, double encoderValue)
{
	//angles are between -.5 and .5
	//This is to allow the motors to rotate in continuous circles (pseudo code on the Team 4048 forum)
	double encPos = encoderValue;
	encPos /= kGearRatio;

	// printf("encoder angle: %f,\n", encPos);

	double temp = angle;
	// temp += (int)encPos;

	encPos = fmod(encPos, 1);

	if ((angle - encPos) > 0.5) temp -= 1;

	if ((angle - encPos) < -0.5) temp += 1;

	return temp;
}

std::string SparkMaxEnclosure::GetName()
{
	return name;
}

void SparkMaxEnclosure::SetReverseEncoder(bool reverseEncoder)
{
	this->reverseEncoder = reverseEncoder;
}

void SparkMaxEnclosure::SetReverseSteerMotor(bool reverseSteer)
{
	this->reverseSteer = reverseSteer;
}
