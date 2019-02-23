package frc.team3130.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import frc.team3130.robot.commands.StartProfile;
import frc.team3130.robot.commands.runMP2019;

public class OI {
    private class JoystickTrigger extends Trigger {

        private Joystick stick;
        private int axis;
        private double threshold;

        private JoystickTrigger(Joystick stick, int axis){
            this.stick = stick;
            this.axis = axis;
            threshold = 0.1;
        }

        private JoystickTrigger(Joystick stick, int axis, double threshold){
            this.stick = stick;
            this.axis = axis;
            this.threshold = threshold;
        }

        @Override
        public boolean get() {
            return stick.getRawAxis(axis) > threshold;
        }

    }

    private class POVTrigger extends Trigger{

        private Joystick stick;
        private int POV;

        public POVTrigger(Joystick stick, int POV) {
            this.stick = stick;
            this.POV = POV;
        }

        @Override
        public boolean get() {
            return stick.getPOV(0)==POV;
        }

    }

    //Instance Handling
    private static OI m_pInstance;
    public static OI GetInstance()
    {
        if(m_pInstance == null) m_pInstance = new OI();
        return m_pInstance;
    }

    /**
     * Definitions for joystick buttons start
     */
    //Joystick
    //public static Joystick stickL;
    //public static Joystick stickR;
    public static Joystick gamepad;

    public static JoystickButton startProfile;
    public static JoystickButton startProfile2019;


    private OI(){
        //stickL = new Joystick(0);
        //stickR = new Joystick(1);
        gamepad = new Joystick(0);

        startProfile = new JoystickButton(gamepad, 1);
        startProfile2019 = new JoystickButton(gamepad, 2);

        startProfile.whenPressed(new StartProfile());
        startProfile2019.whileHeld(new runMP2019());
    }



}
