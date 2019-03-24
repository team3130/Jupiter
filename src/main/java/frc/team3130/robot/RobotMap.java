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

	public static double kChassisWidth = 19.0;
	public static double kTalonTicksPerRotation = 4096;
	public static double kFrameWidth = 19;		// Inches
	public static double kWheelDiameter = 6;	// Inches
	public static double kDistanceToEncoder = kTalonTicksPerRotation / (Math.PI * kWheelDiameter);
	public static double kVelocityToEncoder = kDistanceToEncoder / 10.0; 		// Per 100ms
	public static double kAccelerationToEncoder = kVelocityToEncoder / 10.0; 	// Per 100ms

	public static final int CAN_PNMMODULE = 1;
    public static final int CAN_LEFTMOTOR= 2;
	public static final int CAN_RIGHTMOTOR = 3;

	//Limelight
	public static double kLimelightTiltAngle = 0.53;
	public static double kLimelightForward = 10.5;   // Depth of the camera from the frame pivot (inches)
	public static double kLimelightCalibrateDist = 36;	// Exact horizontal distance between hatch target and lens
	public static double kLimelightHeight = 13.5;
	public static double kLimelightOffset = 4.0;    // Offset to the right side
	public static double kLimelightBumper = 6.0;   // Depth of the camera from the front bumper

	/**
	 * Field parameters
	 */
	public static final double HATCHVISIONTARGET = 31.25;
	public static final double PORTVISIONTARGET = 38.75;
}