#include "drive/SparkTalonEnclosure.h"

SparkTalonEnclosure::SparkTalonEnclosure(std::string name, int moveMotorID, int turnMotorID):
									moveMotor(moveMotorID, rev::CANSparkMax::MotorType::kBrushless), turnMotor(turnMotorID)
{
	this->name = name;

	printf("Spark-Talon Module Initialized: ");
	std::cout << name;
	printf("\tMove: %d Rotate: %d \n", moveMotorID, turnMotorID);

	//clean slate on the motor controllers
	// turnMotor.ConfigFactoryDefault(50);
	//we're going to use the MagEncoder
	turnMotor.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative,0,50);
	//limit the minimum output power on the rotation modules(0 = no real limits)
	turnMotor.ConfigNominalOutputForward(0, 50);
	turnMotor.ConfigNominalOutputReverse(0, 50);
	//Limit the maximum output speed on the rotation modules(1 = no real limits)
	turnMotor.ConfigPeakOutputForward(1, 50);
	turnMotor.ConfigPeakOutputReverse(-1, 50);
	//Adjust for sensor being backwards, it isn't so false
	this->SetReverseEncoder(true);
	//switch which rotational direction is positive, we're fine the way it is.
	this->SetReverseSteerMotor(true);
	//zero the module's rotational encoder, on boot the wheel is at zero degrees
	turnMotor.SetSelectedSensorPosition(0);
	//hitting a very specific target can be hard broaden the target by allowing it to be a little off
	//strykeForce uses 0, not sure if this is what what we think it is
	// turnMotor.ConfigAllowableClosedloopError(0,400,100);

	// turnMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);

	moveMotor.SetInverted(true);

	//PID needs to be setup, likely just P and D, strykeForce uses p=10 and D=100, use tuner
	SetRotationPID(1, 0, 10, 0);
}
SparkTalonEnclosure::~SparkTalonEnclosure(){ return; }

void SparkTalonEnclosure::MoveWheel(double speedVal, double rotationVal, bool optimize)
{

	if(optimize){
		rotationVal = ConvertAngle(rotationVal, GetRawEncoderVal());
	}

	if(IsMoveMotorReversed){
		SetSpeed(-1.0*speedVal);
	}else{
		SetSpeed(speedVal);
	}

	if(speedVal > .01) {
		SetAngle(rotationVal);
	}
	// printf("rotationVal sent to SetAngle:%f", rotationVal);
}

void SparkTalonEnclosure::StopWheel()
{
	moveMotor.StopMotor();
	turnMotor.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0);
}

/////////////////////////utility functions
double SparkTalonEnclosure::ConvertAngle(double targetAngle, double encoderValue)
{

	if(IsMoveMotorReversed){
		if(targetAngle > 0.0){
			targetAngle = targetAngle - 0.5;
		}else{
			targetAngle = 0.5 + targetAngle;
		}
	}
	// convert tick count over to revolutions
	double currentAngle = encoderValue/gearRatio;
	
	// printf("encoder raw: %f, encoder angle: %f, target: %f, ", encoderValue, currentAngle, targetAngle);

	// double temp = targetAngle;
	// temp += trunc(currentAngle);

	//find the current angle in the range (-1,1), in revolutions
	double normalizedCurrentAngle = fmod(currentAngle, 1.0);
	//alternatives if above doesn't work
	// double normalizedCurrentAngle = currentAngle - trunc(currentAngle);
	// double excessCurrentAngle = 0.0;
	// double normalizedCurrentAngle = modf(currentAngle, &excessCurrentAngle);
	// printf("norm CerAng: %f ", normalizedCurrentAngle);

	//force current angle in range [-.5,.5]
	if (normalizedCurrentAngle > 0.5){
		normalizedCurrentAngle -= 1;
	}else if (normalizedCurrentAngle < -0.5){
		normalizedCurrentAngle += 1;
	}
	// printf("norm CerAng Lim: %f, ", normalizedCurrentAngle);
	double errorAngle = targetAngle - normalizedCurrentAngle;
	double distanceToMove = fabs(errorAngle);

	// printf("errAngle1: %f, ", errorAngle);
	if(distanceToMove > .75){
		//complete the circle shortcut
		errorAngle = 1 - errorAngle;
	}else if( (distanceToMove <= .75) && (distanceToMove >= .25) ){
		//invert the motor and angle
		IsMoveMotorReversed = !IsMoveMotorReversed;
		// moveMotor.SetInverted(!moveMotor.GetInverted());
		if(targetAngle > 0.0){
			targetAngle = targetAngle - 0.5;
		}else{
			targetAngle = 0.5 + targetAngle;
		}
		double errorAngle = targetAngle - normalizedCurrentAngle;
	}

	// printf("errAngle: %f\n", errorAngle);
	return errorAngle+currentAngle;
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

	// printf("output sent to Talon:%f", output);
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


bool SparkTalonEnclosure::ShouldReverse(double wa)
{
	double ea = GetRawEncoderVal();
	ea /= gearRatio;
	ea = fmod(ea, 1.0); 

	//Convert the next wheel angle, which is from -.5 to .5, to 0 to 1
	if (wa < 0) wa += .5;

	//Find the difference between the two (not sure if the conversion from (-0.5 to 0.5) to (0 to 1) above is needed)
	//Difference between the two points. May be anything between -1 to 1, but we are looking for a number between -.5 to .5
	double longDifference = fabs(wa - ea);

	//finds shortest distance (0 to 0.5), always positive though (which is what we want)
	double difference = fmin(longDifference, 1.0-longDifference);

	//If the sum is greater than 1/4, then return true (aka it is easier for them to turn around and go backwards than go forward)
	if (difference > 0.25) return true;
	else return false;
}