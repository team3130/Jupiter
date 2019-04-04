package frc.team3130.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Subsystem;

public class Cannon extends Subsystem {
    //Instance Handling
    private static Cannon m_pInstance;

    public static Cannon GetInstance() {
        if (m_pInstance == null) m_pInstance = new Cannon();
        return m_pInstance;
    }

    //Create necessary objects
    private static Relay relay;


    //Create and define all standard data types needed


    private Cannon(){
        /**
         * Constructor:
         * Define and configure your defined objects (ie. talons, vars)
         *
         * _talon.configFactoryDefault();
         * resets hardware defaults that could have been configured on talon before
         *
         */
        relay = new Relay(1);
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    public static void setRelay(Relay.Value set)
    {
        relay.set(set);
    }

}
