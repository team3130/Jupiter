package frc.team3130.robot.commands;

import frc.team3130.robot.OI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;

/**
 *
 */


public class FireCannon extends Command {
    ;

    //DigitalInput relay = new DigitalInput(1);
    Relay relay = new Relay(1);


    public FireCannon() {
        //Put in the instance of whatever subsystem u need here
        //requires();

    }

    // Called just before this Command runs the first time
    protected void initialize() {

        relay.set(Relay.Value.kForward);
        System.out.println ("Did it right");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        relay.set(Value.kOff);
        System.out.println ("hopefully");
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
