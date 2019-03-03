package frc.team3130.robot.commands;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Command;
import frc.team3130.robot.OI;
import frc.team3130.robot.RobotMap;
import frc.team3130.robot.motionProfiling.CubicPath;
import frc.team3130.robot.subsystems.Chassis;
import frc.team3130.robot.vision.Limelight;

public class runMP2019 extends Command {
    private BufferedTrajectoryPointStream pointStreamLeft = new BufferedTrajectoryPointStream();
    private BufferedTrajectoryPointStream pointStreamRight = new BufferedTrajectoryPointStream();

    public runMP2019() {
        requires(Chassis.GetInstance());
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize()
    {
        double maxAcceleration = 30 * RobotMap.kAccelerationToEncoder;
        double cruiseVelocity = 100 * RobotMap.kVelocityToEncoder;

        Limelight.updateData();
        double goStraight = Limelight.getDistanceToTarget(true) - 10;
        double angularOffset = -Math.toRadians(Limelight.getdegHorizontalOffset());
        double goLeft = Math.tan(angularOffset) * goStraight - 3.5;
        double goSlope = -0.0;
        System.out.format("Robot is going to Go %8.3f Left %8.3f %n", goStraight, goLeft);

        /* Convert distances from inches to encoder units */
        goStraight *= RobotMap.kDistanceToEncoder;
        goLeft *= RobotMap.kDistanceToEncoder;

        Chassis.configMP();
        double currentVelocity = Chassis.getVelocity();
        CubicPath path = new CubicPath( maxAcceleration, cruiseVelocity)
                .withDuration(0.1) // 10ms = 0.1 * 100ms
                .withEnterVelocity(currentVelocity)
                .withExitVelocity(0)
                .generateSequence(goStraight, goLeft, goSlope)
                .generateProfiles(RobotMap.kFrameWidth * RobotMap.kDistanceToEncoder);
        int totalCnt = path.getSize();

        /* create an empty point */
        TrajectoryPoint point = new TrajectoryPoint();
        point.headingDeg = 0; /* future feature - not used in this example*/
        point.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
        point.profileSlotSelect1 = 0; /* future feature  - not used in this example - cascaded PID [0,1], leave zero */

        /* Fill up the motion profile's way point streams.
         TODO: Do this inside the path generator instead of populating the profile buffers, maybe? */
        pointStreamLeft.Clear();
        pointStreamRight.Clear();
        for (int i = 0; i < totalCnt; ++i) {
            point.zeroPos = i == 0;
            point.isLastPoint = (i + 1) == totalCnt;

            /* for each point, fill our structure and pass it to API */
            point.position = path.profileLeft[i][0];
            point.velocity = path.profileLeft[i][1];
            point.timeDur = 10;
            pointStreamLeft.Write(point);

            /* for each point, fill our structure and pass it to API */
            point.position = -path.profileRight[i][0];
            point.velocity = -path.profileRight[i][1];
            point.timeDur = 10;
            pointStreamRight.Write(point);
        }

        /* Start motion profiles */
        Chassis.getTalonLeft().startMotionProfile(pointStreamLeft, 5, ControlMode.MotionProfile);
        Chassis.getTalonRight().startMotionProfile(pointStreamRight, 5, ControlMode.MotionProfile);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute()
    {
        Chassis.printVelocity();
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return Chassis.getTalonLeft().isMotionProfileFinished() &&
                Chassis.getTalonRight().isMotionProfileFinished() &&
                !OI.startProfile2019.get();
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
