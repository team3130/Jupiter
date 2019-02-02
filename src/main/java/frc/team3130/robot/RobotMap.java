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


	/**
	 * CAN IDs
	 */
	public static final int CAN_PNMMODULE = 1;

    public static final int CAN_LEFTMOTOR= 2;
	public static final int CAN_RIGHTMOTOR = 3;

//4-10 are CAN ports not IDed yet in Jupiter but are in Juno
	
	public static final int CAN_ELEVATOR1 = 11;
	public static final int CAN_ELEVATOR2 = 12;
	
}