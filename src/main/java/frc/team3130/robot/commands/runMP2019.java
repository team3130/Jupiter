package frc.team3130.robot.commands;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Command;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.motionProfiling.GeneratedMotionProfile;
import frc.team3130.robot.subsystems.Chassis;

public class runMP2019 extends Command {
    public runMP2019() {
        requires(Chassis.GetInstance());
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
        double[][] profile = GeneratedMotionProfile.Points;
        int totalCnt = GeneratedMotionProfile.kNumPoints;
        /* create an empty point */
        TrajectoryPoint point = new TrajectoryPoint();
        BufferedTrajectoryPointStream pointStream = new BufferedTrajectoryPointStream();
        for (int i = 0; i < totalCnt; ++i) {
            double positionRot = profile[i][0];
            double velocityRPM = profile[i][1];
            /* for each point, fill our structure and pass it to API */
            point.position = positionRot * RobotMap.kTalonTicksPerRotation; //Convert Revolutions to Units
            point.velocity = velocityRPM * RobotMap.kTalonTicksPerRotation / 600.0; //Convert RPM to Units/100ms
            point.headingDeg = 0; /* future feature - not used in this example*/
            point.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
            point.profileSlotSelect1 = 0; /* future feature  - not used in this example - cascaded PID [0,1], leave zero */
            point.timeDur = (int)profile[i][2];
            point.zeroPos = i == 0;
            point.isLastPoint = (i + 1) == totalCnt;

            pointStream.Write(point);
        }
        Chassis.configMP();
        Chassis.getTalon().startMotionProfile(pointStream, 5, ControlMode.MotionProfile);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        Chassis.printVelocity();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return Chassis.getTalon().isMotionProfileFinished();
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Chassis.getTalon().set(ControlMode.PercentOutput, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }

}
