package frc.team3130.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.commands.DefaultDrive;

public class Chassis extends Subsystem{

    //Instance Handling
    private static Chassis m_pInstance;
    public static Chassis GetInstance()
    {
        if(m_pInstance == null) m_pInstance = new Chassis();
        return m_pInstance;
    }

    private static DifferentialDrive m_drive;

    private static WPI_TalonSRX m_leftMotor;
    private static WPI_TalonSRX m_rightMotor;




    private Chassis() {
        m_leftMotor = new WPI_TalonSRX(RobotMap.CAN_LEFTMOTOR);
    	m_rightMotor = new WPI_TalonSRX(RobotMap.CAN_RIGHTMOTOR);

    	m_leftMotor.configFactoryDefault(FeedbackDevice.CTRE_MagEncoder_Relative.value);
    	m_rightMotor.configFactoryDefault(FeedbackDevice.CTRE_MagEncoder_Relative.value);


        m_leftMotor.setNeutralMode(NeutralMode.Brake);
        m_rightMotor.setNeutralMode(NeutralMode.Brake);


        
        m_drive = new DifferentialDrive(m_leftMotor, m_rightMotor);
        m_drive.setSafetyEnabled(false);

    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        setDefaultCommand(new DefaultDrive());
        //setDefaultCommand(new MySpecialCommand());
    }

    public static void DriveArcade(double moveThrottle, double turnThrottle, boolean squaredinputs){
        m_drive.arcadeDrive(moveThrottle, turnThrottle, squaredinputs);
    }

    public static WPI_TalonSRX getTalon(){
        return m_leftMotor;
    }



}
