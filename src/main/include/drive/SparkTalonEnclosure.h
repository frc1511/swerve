#pragma once

#include "frc/WPILib.h"
#include "drive/SwerveEnclosure.h"
#include "ctre/Phoenix.h"
#include "ctre/phoenix/MotorControl/CAN/TalonSRX.h"
#include "rev/CANSparkMax.h"

#include <math.h>

/**
 * Used for enclosing a CANTalon speed controller (rotational movement) and a
 * Spark Max speed controller (directional movement) in order to control a single
 * swerve module .  The CANTalon speed controller must have an encoder wired to it in
 * order for this enclosure to function. This intent is for a Mag encoder.
 *
 * This class inherits from the SwerveEnclosure class, used by
 * RobotDriveSwerve objects in order for easy control of the swerve system. This
 * class can be inherited from to allow for modification and support different
 * hardware designs.
 *
 * This enclosure requires CTRE's CANTalon libraries in order to be used.
 */
class SparkTalonEnclosure : public SwerveEnclosure {

public:
	enum MotorType{
		MoveMotor,
		TurnMotor
	};

	/*
	 * Requires any speed controller for controlling wheel direction, a CANTalon
	 * controller to control wheel rotation, and a gear ratio value.  The default
	 * gear ratio is 3868.44.
	 */
	SparkTalonEnclosure(	std::string name,
					int moveMotor,
					int turnMotor);
	~SparkTalonEnclosure();

	/**
	 * Move the wheel to the given speed and rotational values.
	 * Method is used by RobotDriveSwerve for controlling each individual swerve
	 * wheel.
	 * @param speedVal The speed of the moveMotor with input range 0 to 1 for PercentVBus mode 
	 * @param rotationVal The rotational position of the swerve module in units of revolutions of the module
	 * @param optimize whether or not to limit angles to -0.5 to 0.5 and making a shortcut by reversing the move motors
	 */
	void MoveWheel(double speedVal, double rotationVal, bool optimize) override;
	
	/**
	 * Stops all movement of an enclosures motors
	 */
	void StopWheel() override;
	
	/**
	 * Sets the PIDF value being used by a swerve enclosure. If all values are
	 * zero the swerve module will not spin. Thesevalues must be tuned to optimize
	 * the modules turn while under proper weight and on the appropriate driving
	 * surface.
	 */
	void SetRotationPID(double P, double I, double D, double F = 0);
	/*
	 * Outputs value of enclosures encoder as double in units of revolutions
	 */
	double GetRotationalPos() override;
	/*
	 * Outputs value of enclosures encoder in units of ticks
	 */
	double GetRawEncoderVal() override;
	/*
	 * Returns the name of the enclosure
	 */
	std::string GetName() override;
	
	/**
	 * Reverse encoder direction for the turnMotor. This should not be used 
	 * to switch the direction of the turnMotor encoder with respect to the
	 * turnMotor output. Instead us
	 */
	void SetReverseEncoder(bool reverseEncoder);
	/*
	 * Reverse steer motor direction
	 */
	void SetReverseSteerMotor(bool reverseSteer);
private:
	/*
	 * Using the desired angle for the wheel and the current encoder position,
	 * it determines if the wheel could be efficient by reversing the rotation
	 * and movement direction.
	 */
	// bool ShouldReverse(double desiredPos);
	/*
	 * Sets the speed value to the movement motor
	 */
	void SetSpeed(double speedVal);
	
	/**
	 * Sets the that the wheel should turn to using the TalonSRX's internal PID loop.
	 * @param 
	 */
	void SetAngle(double rotationVal);
	
	/**
	 * Converts the given angle to a range of -0.5 to 0.5. Remember that one
	 * rotation of the module is an angle of 1, so the range -0.5 to 0.5 is 
	 * one full rotation of the module
	 * @param targetAngle The angle that the module should turn to. In units of revolutions
	 * @param encoderValue The tick count of the encoders. In units of ticks
	 * @return angle that the module should turn to, in a range of -.0.5 to 0.5, in units of revolutions
	 */
	double ConvertAngle(double targetAngle, double encoderValue);

	public:
	rev::CANSparkMax moveMotor;
	private:
	ctre::phoenix::motorcontrol::can::TalonSRX turnMotor;

	std::string name;
	// The 2019 swerve prototype has a gear 18 teeth on the driving motor output.
	// The 2019 swerve prototype has 68 teeth on the driven/rotating module.
	// The 2019 swerve prototype uses a Vex mag encoder whihc has 1024 ticks per rotation.
	// The constant kGearRatio is in units of encoder ticks per module revolution. I.E. 1024*68/18 for 2019 prototype 
	double gearRatio = 3868.44*4;//not sure why we are multipling by 4
	
};

