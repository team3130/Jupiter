package frc.team3130.robot.commands;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Command;
import frc.team3130.robot.OI;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.motionProfiling.CubicPath;
import frc.team3130.robot.subsystems.Chassis;

public class runMP2019 extends Command {
    BufferedTrajectoryPointStream pointStreamLeft = new BufferedTrajectoryPointStream();
    BufferedTrajectoryPointStream pointStreamRight = new BufferedTrajectoryPointStream();

    public runMP2019() {
        requires(Chassis.GetInstance());
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
        Chassis.configMP();
        // Lets use wheel revolutions for the distance and rev/sec for velocity for now
        // Then the robot width is 19" / (6" x pi) = 1.007 (surprise!)
        CubicPath path = new CubicPath(1, 2)
                .withEnterVelocity(0)
                .withExitVelocity(0)
                .withDestination(5, 0, 0.0)
                .generateSequence(0.01)
                .generateProfiles(1.007);
        int totalCnt = path.size();
        pointStreamLeft = new BufferedTrajectoryPointStream();
        pointStreamRight = new BufferedTrajectoryPointStream();
        /* create an empty point */
        TrajectoryPoint point = new TrajectoryPoint();
        point.headingDeg = 0; /* future feature - not used in this example*/
        point.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
        point.profileSlotSelect1 = 0; /* future feature  - not used in this example - cascaded PID [0,1], leave zero */
        for (int i = 0; i < totalCnt; ++i) {
            point.zeroPos = i == 0;
            point.isLastPoint = (i + 1) == totalCnt;

            /* for each point, fill our structure and pass it to API */
            point.position = path.profileLeft[i][0] * RobotMap.kTalonTicksPerRotation; //Convert Revolutions to Units
            point.velocity = path.profileLeft[i][1] * RobotMap.kTalonTicksPerRotation / 600.0; //Convert RPM to Units/100ms
            point.timeDur = (int)path.profileLeft[i][2];
            pointStreamLeft.Write(point);

            /* for each point, fill our structure and pass it to API */
            point.position = path.profileRight[i][0] * RobotMap.kTalonTicksPerRotation; //Convert Revolutions to Units
            point.velocity = path.profileRight[i][1] * RobotMap.kTalonTicksPerRotation / 600.0; //Convert RPM to Units/100ms
            point.timeDur = (int)path.profileRight[i][2];
            pointStreamRight.Write(point);
        }
        Chassis.getTalonLeft().startMotionProfile(pointStreamLeft, 5, ControlMode.MotionProfile);
        Chassis.getTalonRight().startMotionProfile(pointStreamRight, 5, ControlMode.MotionProfile);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Chassis.printVelocity();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return Chassis.getTalonLeft().isMotionProfileFinished() &&
                Chassis.getTalonRight().isMotionProfileFinished() &&
                !OI.gamepad.getRawButton(2);
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Chassis.getTalonLeft().set(ControlMode.PercentOutput, 0);
        Chassis.getTalonRight().set(ControlMode.PercentOutput, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }

}
