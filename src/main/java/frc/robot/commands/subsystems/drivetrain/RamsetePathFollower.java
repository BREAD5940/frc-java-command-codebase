package frc.robot.commands.subsystems.drivetrain;

import java.io.File;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.math.Util;
import frc.robot.Robot;
import frc.robot.lib.Logger;
import frc.robot.lib.enums.MotionProfileDirection;
import frc.robot.lib.motion.Odometer;
import frc.robot.lib.motion.followers.RamseteFollower;
import frc.robot.lib.obj.DriveSignal;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

@SuppressWarnings("FieldCanBeLocal")
public class RamsetePathFollower extends Command{

    private RamseteFollower ramseteFollower;
    private DriveSignal driveSignal;
    private Trajectory.Segment current;
    private Trajectory path;

    public RamsetePathFollower(Trajectory trajectory){
        this(trajectory, MotionProfileDirection.FORWARD, 15);
    }

    public RamsetePathFollower(Trajectory trajectory, MotionProfileDirection direction, double timeout){
        requires(Robot.drivetrain);
        // ramseteFollower = new RamseteFollower(trajectory, direction);
        setTimeout(timeout);
        path = trajectory;
    }

    public RamsetePathFollower(Trajectory trajectory, double b, double zeta, MotionProfileDirection direction, double timeout){
        requires(Robot.drivetrain);
        // ramseteFollower = new RamseteFollower(trajectory, b, zeta, direction);
        setTimeout(timeout);
        path = trajectory;
    }

    public RamsetePathFollower(String filepath) {
      this(Pathfinder.readFromCSV( new File("/home/lvuser/deploy/paths/test.pf1.csv")), MotionProfileDirection.FORWARD, 15 );
    }

    @Override
    protected void initialize(){
        Robot.drivetrain.setMode(NeutralMode.Brake);
        Robot.drivetrain.zeroGyro();
        Odometer.getInstance().reverseXY(true);
        Odometer.getInstance().setOdometryForPathfinder(path);
    }

    @Override
    protected void execute(){
        driveSignal = ramseteFollower.getNextDriveSignal();
        current = ramseteFollower.currentSegment();

        // double leftVelocity = Util.toFeet(driveSignal.getL());
        // double rightVelocity = Util.toFeet(driveSignal.getR());

        Logger.log("--------------------------");
        // Logger.log("left velocity: " + leftVelocity + " right velocity: " + rightVelocity);

        // Robot.drivetrain.setVelocity(driveSignal.getL(), driveSignal.getR(), current.acceleration);
        // Robot.drivetrain.setFeetPerSecondArbitraryFeedForward(leftVelocity, rightVelocity, current.acceleration);
        // Robot.drivetrain.setFeetPerSecond(leftVelocity, rightVelocity);

        // OI.liveDashboardTable.getEntry("Path X").setNumber(current.x);
        // OI.liveDashboardTable.getEntry("Path Y").setNumber(current.y);

        SmartDashboard.putNumber("Path X", current.x);
        SmartDashboard.putNumber("Path Y", current.y);

        // SmartDashboard.putNumber("Path Left Wheel Velocity", driveSignal.getL());
        // SmartDashboard.putNumber("Path Right Wheel Velocity", driveSignal.getR());

        SmartDashboard.putNumber("Robot Left Velocity", Util.toMeters(Robot.drivetrain.getLeftVelocity()));
        SmartDashboard.putNumber("Robot Right Velocity", Util.toMeters(Robot.drivetrain.getRightVelocity()));
    }

    @Override
    protected boolean isFinished() {
      return isTimedOut() || ramseteFollower.isFinished();
    }

    @Override
    protected void end(){
        Robot.drivetrain.setSpeeds(0, 0);

        Odometer.getInstance().reverseXY(false);
    }

  }
