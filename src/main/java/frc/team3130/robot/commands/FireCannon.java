package frc.team3130.robot.commands;

import frc.team3130.robot.OI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.subsystems.Cannon;
import frc.team3130.robot.OI;

import static frc.team3130.robot.OI.fireCannon;

/**
 *
 */


public class FireCannon extends Command {
    ;

    //DigitalInput relay = new DigitalInput(1);
    



    public FireCannon() {
        requires(Cannon.GetInstance());

    }

    // Called just before this Command runs the first time
    protected void initialize() {


    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
        Cannon.setRelay(Relay.Value.kOn);
        System.out.println ("Did it ton");

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
       return false;
    }

    // Called once after isFinished returns true
    protected void end() {
        Cannon.setRelay(Relay.Value.kOff);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
        end();
    }
}
