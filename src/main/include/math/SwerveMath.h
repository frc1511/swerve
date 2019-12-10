// this file was originally written by FRC4048
// https://github.com/FRC4048/Swerve-Drive-Library-Cpp

#pragma once
#include <math.h>
#include "frc/WPILib.h"

/**
 * This class handles the calculations required to drive a robot using SwerveDrive
 * wheels.  This class supports both robot centric and field centric modes.  Field
 * centric mode causes the swerve system to always keep the forward/backward and
 * directional movement to stay relative to zero degrees of the gyro input via 
 * angle.  Robot centric mode causes the swerve system to act with normal cartesian
 * movement with respect to the front of the robot.
 */
class SwerveMath {

public:
	/**
	 * Requires the length and width between swerve wheels in order to be 
	 * accurate with calculations. The units of distance do not matter as long
	 * as both length and width are in the same units.
	 *
	 * @param length the distance between the front two wheels
	 * @param width the distance between the front and back wheels
	 */
	SwerveMath(double length, double width);

	/**
	 * Uses foward speed, strafing speed, and rotational speed values to calculate
	 * the required angle and speed for each wheel.  An angle can also be given so
	 * that field centric mode can be used.  If no angle is given (or equal to -999)
	 * robot centric will be used.
	 *
	 * @param fwd FORWARD: -1 to 1 positive = forward movement, negative value = backward movement
	 * @param str STRAFE: -1 to 1 positive = right direction, negative value = left direction
	 * @param rot ROTATION: -1 to 1 positive value = clockwise rotation, negative value = counterclockwise rotation
	 * @param angle value of a gyro input into the function for field centric
	 *
	 * @return array of speed and rotation value for each wheel. Follows this form:
	 * 		0			1
	 * 	0	Front Left Speed	Front Left Angle
	 * 	1	Front Right Speed	Front Right Angle
	 * 	2	Rear Left Speed		Rear Left Angle
	 * 	3	Rear Right Speed	Rear Right Angle
	 */
	double** Calculate(double fwd, double str, double rot, double angle = 0.0 );

private:
	static constexpr double NO_ANGLE = 0.0;//Default value for robot centric drive
	static constexpr double PI = acos(-1.0);//This is the value of PI

	/*
	 * Copies the speed and angle values into a pointer in order to be used by
	 * the enclosures
	 */
	double** CopyArray(double array[][2]);

	double LENGTH, WIDTH;//storage of the length and width values
	double R;//the hypotenuse of width and length
};
