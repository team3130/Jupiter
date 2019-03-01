/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3130.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	public static double kTalonTicksPerRotation = 4096;
	public static double kFrameWidth = 19;		// Inches
	public static double kWheelDiameter = 6;	// Inches
	public static double kCruiseVelocity = 100;	// Inches per second
	public static double kMaxAcceleration = 30;	// Inches per second per second
	public static double kDistanceToEncoder = kTalonTicksPerRotation / (Math.PI * kWheelDiameter);
	public static double kVelocityToEncoder = kTalonTicksPerRotation / (Math.PI * kWheelDiameter) / 10.0; // Per 100ms
	public static double kAccelerationToEncoder = kTalonTicksPerRotation / (Math.PI * kWheelDiameter) / 100.0;

	public static final int CAN_PNMMODULE = 1;

    public static final int CAN_LEFTMOTOR= 2;
	public static final int CAN_RIGHTMOTOR = 3;

	
}