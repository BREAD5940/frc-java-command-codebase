package frc.robot.commands.subsystems.drivetrain;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;

import org.team5940.pantry.experimental.command.SendableCommandBase;
import frc.robot.Robot;

public class SetInitialOdometry extends SendableCommandBase {

	Pose2dWithCurvature initialPose;

	public SetInitialOdometry(TimedTrajectory<Pose2dWithCurvature> trajectory) {
		requires(Robot.drivetrain);
		initialPose = trajectory.getFirstState().getState();
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		// Robot.drivetrain.getLocalization().reset(initialPose.getPose());
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {}
}
