#include "drive/SparkTalonEnclosure.h"

SparkTalonEnclosure::SparkTalonEnclosure(std::string name, int moveMotorID, int turnMotorID):
									moveMotor(moveMotorID, rev::CANSparkMax::MotorType::kBrushless), turnMotor(turnMotorID)
{
	this->name = name;

	printf("Spark-Talon Module Initialized: "+name+" Move: %f Rotate: %f", moveMotorID, turnMotorID);

	//clean slate on the motor controllers
	turnMotor.ConfigFactoryDefault(50);
	//we're going to use the MagEncoder
	turnMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0,50);
	//limit the minimum output power on the rotation modules(0 = no real limits)
	turnMotor.ConfigNominalOutputForward(0, 50);
	turnMotor.ConfigNominalOutputReverse(0, 50);
	//Limit the maximum output speed on the rotation modules(1 = no real limits)
	turnMotor.ConfigPeakOutputForward(1, 50);
	turnMotor.ConfigPeakOutputReverse(-1, 50);
	//Adjust for sensor being backwards, it isn't so false
	this->SetReverseEncoder(false);
	//switch which rotational direction is positive, we're fin the way it is.
	this->SetReverseSteerMotor(false);
	//zero the module's rotational encoder, on boot the wheel is at zero degrees
	turnMotor.SetSelectedSensorPosition(0);
	//hitting a very specific target can be hard broaden the target by allowing it to be a little off
	turnMotor.ConfigAllowableClosedloopError(0,gearRatio*3/360,100);

	SetPID(1, 0, 0, .1);
}
SparkTalonEnclosure::~SparkTalonEnclosure(){ return; }

void SparkTalonEnclosure::MoveWheel(double speedVal, double rotationVal, bool optimize)
{
	if(optimize){
		//currently working without this section
		// rotationVal = ConvertAngle(rotationVal, GetEncoderVal());
		// if(ShouldReverse(rotationVal))
		// {
		// 	if(rotationVal < 0)
		// 		rotationVal += 0.5;
		// 	else
		// 		rotationVal -= 0.5;

		// 	speedVal *= -1;
		// }
	}

	SetSpeed(speedVal);
	//removed moment requirement in order to turn, for now.
	SetAngle(rotationVal);

	// printf("moving %f, %f\n", speedVal, rotationVal);
}

void SparkTalonEnclosure::StopWheel()
{
	moveMotor.StopMotor();
	turnMotor.StopMotor();
}


/////////////////////////modifier outputs
void SparkMaxEnclosure::SetSpeed(double speedVal)
{
	moveMotor.Set(speedVal); 
}

void SparkTalonEnclosure::SetAngle(double desiredAngle)
{
	double output = desiredAngle*gearRatio;

	turnMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, output);

}

//////////////////////////accessor
double SparkTalonEnclosure::GetRotationalPos()
{
	return turnMotor.GetSelectedSensorPosition()/gearRatio;
}

std::string SparkTalonEnclosure::GetName()
{
	return name;
}

//////////////////////////setup

void SparkTalonEnclosure::SetRotationPID(double P, double I, double D, double F) {
	turnMotor.Config_kP(0, P, 100);
	turnMotor.Config_kI(0, I, 100);
	turnMotor.Config_kD(0, D, 100);
	turnMotor.Config_kF(0, F, 100);
}

void SparkTalonEnclosure::SetReverseEncoder(bool reverseEncoder)
{
	turnMotor.SetSensorPhase(reverseEncoder);
}

void SparkTalonEnclosure::SetReverseSteerMotor(bool reverseSteer)
{
	turnMotor.SetInverted(reverseSteer);
}
