package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.commands.DefaultDrive;
import frc.team3130.robot.util.Util;


public class Chassis extends Subsystem{

    //Instance Handling
    private static Chassis m_pInstance;
    public static Chassis GetInstance()
    {
        if(m_pInstance == null) m_pInstance = new Chassis();
        return m_pInstance;
    }

    private static DifferentialDrive m_drive;

    private static WPI_TalonSRX m_leftMotorFront;
    private static WPI_TalonSRX m_leftMotorRear;
    private static WPI_TalonSRX m_rightMotorFront;
    private static WPI_TalonSRX m_rightMotorRear;
    private static WPI_TalonSRX m_cannon;




    private Chassis() {
        m_leftMotorFront = new WPI_TalonSRX(RobotMap.CAN_LEFTMOTORFRONT);
        m_leftMotorRear = new WPI_TalonSRX(RobotMap.CAN_LEFTMOTORREAR);
        m_rightMotorFront = new WPI_TalonSRX(RobotMap.CAN_RIGHTMOTORFRONT);
        m_rightMotorRear = new WPI_TalonSRX(RobotMap.CAN_RIGHTMOTORREAR);
        m_cannon = new WPI_TalonSRX(RobotMap.CAN_CANNON);

        m_leftMotorFront.configFactoryDefault();
        m_leftMotorRear.configFactoryDefault();
        m_rightMotorFront.configFactoryDefault();
        m_rightMotorRear.configFactoryDefault();
        m_cannon.configFactoryDefault();

        m_cannon.configVoltageCompSaturation(12.0, 0);
        m_cannon.enableVoltageCompensation(true);

        m_leftMotorFront.setNeutralMode(NeutralMode.Brake);
        m_rightMotorFront.setNeutralMode(NeutralMode.Brake);
        m_leftMotorRear.setNeutralMode(NeutralMode.Brake);
        m_rightMotorRear.setNeutralMode(NeutralMode.Brake);


        m_leftMotorRear.set(ControlMode.Follower, RobotMap.CAN_LEFTMOTORFRONT);
        m_rightMotorRear.set(ControlMode.Follower, RobotMap.CAN_RIGHTMOTORFRONT);




        /**
         * For all motors, forward is the positive direction
         *
         * Shift false is low gear
         */

        m_rightMotorFront.setInverted(true);
        m_leftMotorFront.setInverted(false);
        m_rightMotorRear.setInverted(true);
        m_leftMotorRear.setInverted(false);

        m_rightMotorFront.setSensorPhase(false);

        m_leftMotorFront.overrideLimitSwitchesEnable(false);
        m_rightMotorFront.overrideLimitSwitchesEnable(false);

    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new DefaultDrive());
        //setDefaultCommand(new MySpecialCommand());
    }

    public static void driveArcade(double moveThrottle, double turnThrottle, boolean squaredInputs) {
        moveThrottle = Util.limit(moveThrottle, 1.0);
        moveThrottle = Util.applyDeadband(moveThrottle, RobotMap.kDriveDeadband);

        turnThrottle = Util.limit(turnThrottle, 1.0);
        turnThrottle = Util.applyDeadband(turnThrottle, RobotMap.kDriveDeadband);

        if(squaredInputs){
            moveThrottle = Math.copySign(moveThrottle * moveThrottle, moveThrottle);
            turnThrottle = Math.copySign(turnThrottle * turnThrottle, turnThrottle);
        }

        double leftMotorOutput = moveThrottle + turnThrottle;
        double rightMotorOutput = moveThrottle - turnThrottle;

        m_leftMotorFront.set(ControlMode.PercentOutput, Util.limit(leftMotorOutput, 1.0));
        m_rightMotorFront.set(ControlMode.PercentOutput, Util.limit(rightMotorOutput, 1.0));
    }






    /**
     * Gets absolute distance traveled by the left side of the robot
     * @return The absolute distance of the left side in inches
     */
    public static double getDistanceL()
    {
        return m_leftMotorFront.getSelectedSensorPosition(0) / RobotMap.kTalonTicksPerRotation;
    }

    /**
     * Gets absolute distance traveled by the right side of the robot
     * @return The absolute distance of the right side in inches
     */
    public static double getDistanceR()
    {
        return m_rightMotorFront.getSelectedSensorPosition(0) / RobotMap.kTalonTicksPerRotation;
    }

    /**
     * Gets the absolute distance traveled by the robot
     * @return The absolute distance traveled of robot in inches
     */
    public static double getDistance()
    {
        return (getDistanceL() + getDistanceR()) / 2.0; //the average of the left and right distances
    }

    /**
     * Returns the current speed of the front left motor in native units
     * @return Current speed of the front left motor (ticks per 0.1 seconds)
     */
    public static double getRawSpeedL()
    {
        return m_leftMotorFront.getSelectedSensorVelocity(0);
    }

    /**
     * Returns the current speed of the front left motor in native units
     * @return Current speed of the front left motor (ticks per 0.1 seconds)
     */
    public static double getRawSpeedR()
    {
        return m_rightMotorFront.getSelectedSensorVelocity(0);
    }

    /**
     * Returns the current speed of the front left motor
     * @return Current speed of the front left motor (inches per second)
     */
    public static double getSpeedL()
    {
        // The raw speed units will be in the sensor's native ticks per 100ms.
        return 10.0 * getRawSpeedL() / RobotMap.kTalonTicksPerRotation;
    }

    /**
     * Returns the current speed of the front right motor
     * @return Current speed of the front right motor (inches per second)
     */
    public static double getSpeedR()
    {
        // The raw speed units will be in the sensor's native ticks per 100ms.
        return 10.0 * getRawSpeedR() / RobotMap.kTalonTicksPerRotation;
    }

    /**
     * Returns the current speed of the robot by averaging the front left and right motors
     * @return Current speed of the robot
     */
    public static double getSpeed()
    {
        return 0.5 * (getSpeedL() + getSpeedR());
    }

    /**
     *
     * @return Raw absolute encoder ticks of the left side of the robot
     */
    public static double getRawL(){
        return m_leftMotorFront.getSelectedSensorPosition(0);
    }

    /**
     *
     * @return Raw absolute encoder ticks of the right side of the robot
     */
    public static double getRawR(){
        return m_rightMotorFront.getSelectedSensorPosition(0);
    }

    /**
     *
     * @return Returns the left main drive Talon
     */
    public static WPI_TalonSRX getFrontL(){
        return m_leftMotorFront;
    }

    /**
     *
     * @return Returns the right main drive Talon
     */
    public static WPI_TalonSRX getFrontR(){ return m_rightMotorFront; }


}
