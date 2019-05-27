package frc.robot.commands.subsystems.drivetrain;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.team5940.pantry.exparimental.command.SendableCommandBase;

import frc.robot.Robot;

public class SetInitialOdometry extends SendableCommandBase {

	Pose2dWithCurvature initialPose;

	public SetInitialOdometry(TimedTrajectory<Pose2dWithCurvature> trajectory) {
		addRequirements(Robot.drivetrain);
		initialPose = trajectory.getFirstState().getState();
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		// Robot.drivetrain.getLocalization().reset(initialPose.getPose());
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	// @Override
	// protected void interrupted() {}
}
