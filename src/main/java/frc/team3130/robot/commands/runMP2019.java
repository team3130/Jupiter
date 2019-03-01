package frc.team3130.robot.commands;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.team3130.robot.OI;
import frc.team3130.robot.Robot;
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
        double maxAcceleration = RobotMap.kMaxAcceleration * RobotMap.kAccelerationToEncoder;
        double cruiseVelocity = RobotMap.kCruiseVelocity * RobotMap.kVelocityToEncoder;
        double timeStart = Timer.getFPGATimestamp();
        CubicPath path = new CubicPath( maxAcceleration, cruiseVelocity);
        path.withDuration(0.1); // 10ms = 0.1 * 100ms
        path.generateSequence(50*RobotMap.kDistanceToEncoder, 15*RobotMap.kDistanceToEncoder, 0.5); // Inches
        path.generateProfiles(RobotMap.kFrameWidth * RobotMap.kDistanceToEncoder);
        int totalCnt = path.getSize();
        pointStreamLeft = new BufferedTrajectoryPointStream();
        pointStreamRight = new BufferedTrajectoryPointStream();
        //System.out.format("Time to create buffers: %f%n", Timer.getFPGATimestamp() - timeStart);
        /* create an empty point */
        TrajectoryPoint point = new TrajectoryPoint();
        point.headingDeg = 0; /* future feature - not used in this example*/
        point.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
        point.profileSlotSelect1 = 0; /* future feature  - not used in this example - cascaded PID [0,1], leave zero */
        for (int i = 0; i < totalCnt; ++i) {
            point.zeroPos = i == 0;
            point.isLastPoint = (i + 1) == totalCnt;

            System.out.format("LP %8.3f  RP %8.3f | LV %8.3f  RV %8.3f%n",
                    path.profileLeft[i][0], path.profileRight[i][0],
                    path.profileLeft[i][1], path.profileRight[i][1]);
            /* for each point, fill our structure and pass it to API */
            point.position = path.profileLeft[i][0];
            point.velocity = path.profileLeft[i][1];
            point.timeDur = 10; //(int)path.profileLeft[i][2];
            pointStreamLeft.Write(point);

            /* for each point, fill our structure and pass it to API */
            point.position = -path.profileRight[i][0];
            point.velocity = -path.profileRight[i][1];
            point.timeDur = 10; //(int)path.profileRight[i][2];
            pointStreamRight.Write(point);
        }
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
