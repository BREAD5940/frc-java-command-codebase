package frc.robot.commands.auto.actions;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.lib.enums.MotionProfileDirection;
import frc.robot.lib.motion.DriveSignal;
import frc.robot.lib.motion.followers.RamseteFollower;
import jaci.pathfinder.Trajectory;

@SuppressWarnings("FieldCanBeLocal")
public class RamsetePathFollower extends Command{

    private RamseteFollower ramseteFollower;
    private DriveSignal driveSignal;
    private Trajectory.Segment current;

    public RamsetePathFollower(Trajectory trajectory){
        this(trajectory, MotionProfileDirection.FORWARD);
    }

    public RamsetePathFollower(Trajectory trajectory, MotionProfileDirection direction){
        requires(Robot.drivetrain);
        ramseteFollower = new RamseteFollower(trajectory, direction);
    }

    public RamsetePathFollower(Trajectory trajectory, double b, double zeta, MotionProfileDirection direction){
        requires(Robot.drivetrain);
        ramseteFollower = new RamseteFollower(trajectory, b, zeta, direction);
    }

    @Override
    protected void initialize(){
        Robot.drivetrain.setMode(NeutralMode.Brake);
    }

    @Override
    protected void execute(){
        driveSignal = ramseteFollower.getNextDriveSignal();
        current = ramseteFollower.currentSegment();

        Robot.drivetrain.setFeetPerSecond(driveSignal.getLeft(), driveSignal.getRight(), current.acceleration);

        SmartDashboard.putNumber("Path X: ", current.x);
        SmartDashboard.putNumber("Path Y: ", current.y);

        SmartDashboard.putNumber("Path Left Wheel Velocity", driveSignal.getLeft());
        SmartDashboard.putNumber("Path Right Wheel Velocity", driveSignal.getRight());

        SmartDashboard.putNumber("Robot Left Velocity", Robot.drivetrain.getLeftVelocity());
        SmartDashboard.putNumber("Robot Right Velocity",Robot.drivetrain.getRightVelocity());
    }

    @Override
    protected void end(){
        Robot.drivetrain.setSpeeds(0, 0);
    }

    @Override
    protected boolean isFinished(){
        return ramseteFollower.isFinished();
    }
}