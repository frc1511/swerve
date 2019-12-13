#include "drive/SparkTalonEnclosure.h"

SparkTalonEnclosure::SparkTalonEnclosure(std::string name, int moveMotorID, int turnMotorID):
									moveMotor(moveMotorID, rev::CANSparkMax::MotorType::kBrushless), turnMotor(turnMotorID)
{
	this->name = name;

	printf("Spark-Talon Module Initialized: ");
	std::cout << name;
	printf("\tMove: %d Rotate: %d \n", moveMotorID, turnMotorID);

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
	//switch which rotational direction is positive, we're fine the way it is.
	this->SetReverseSteerMotor(true);
	//zero the module's rotational encoder, on boot the wheel is at zero degrees
	turnMotor.SetSelectedSensorPosition(0);
	//hitting a very specific target can be hard broaden the target by allowing it to be a little off
	//strykeForce uses 0, not sure if this is what what we think it is
	turnMotor.ConfigAllowableClosedloopError(0,400,100);

	turnMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);

	//PID needs to be setup, likely just P and D, strykeForce uses p=10 and D=100, use tuner
	SetRotationPID(.05, 0, .5, 0);
}
SparkTalonEnclosure::~SparkTalonEnclosure(){ return; }

void SparkTalonEnclosure::MoveWheel(double speedVal, double rotationVal, bool optimize)
{
	rotationVal = ConvertAngle(rotationVal, GetRawEncoderVal());

	if(optimize){
		//currently working without this section
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
	//removed movement requirement in order to turn, for now.
	SetAngle(rotationVal);

	printf("rotationVal sent to SetAngle:%f", rotationVal);
}

void SparkTalonEnclosure::StopWheel()
{
	moveMotor.StopMotor();
	turnMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
}

/////////////////////////utility functions
double SparkTalonEnclosure::ConvertAngle(double targetAngle, double encoderValue)
{

	//this seems to result in an inverted angle, find out why!!!!!!!!

	//angles are between -.5 and .5
	//This is to allow the motors to rotate in continuous circles 
	double currentAngle = encoderValue/gearRatio;
	
	// printf("encoder angle raw: %f, encoder angle mapped: %f, desired angle: %f\n", encoderValue, encPos, angle);

	double temp = targetAngle;
	temp += (int)currentAngle;

	currentAngle = fmod(currentAngle, 1);
	// printf("encoder angle raw: %f, encoder angle mapped: %f, desired angle: %f, angle-encPos: %f\n", encoderValue, encPos, angle, (angle - encPos));

	if ((targetAngle - currentAngle) > 0.5){
		temp -= 1;
	}else if ((targetAngle - currentAngle) < -0.5){
		temp += 1;
	}

	return temp;
}
/////////////////////////modifier outputs
void SparkTalonEnclosure::SetSpeed(double speedVal)
{
	moveMotor.Set(speedVal); 
}

void SparkTalonEnclosure::SetAngle(double rotationSetpoint)
{
	//convert units of module revolutions to tick counts
	double output = rotationSetpoint*gearRatio;

	printf("output sent to Talon:%f", output);
	//set a setpoint to the motor controller
	turnMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Position, output);
}

//////////////////////////accessor
double SparkTalonEnclosure::GetRawEncoderVal(){
	return turnMotor.GetSelectedSensorPosition();
}

double SparkTalonEnclosure::GetRotationalPos()
{
	return this->GetRawEncoderVal()/gearRatio;
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
