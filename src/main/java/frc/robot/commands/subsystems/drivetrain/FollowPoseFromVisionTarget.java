package frc.robot.commands.subsystems.drivetrain;

import org.team5940.pantry.exparimental.command.SendableCommandBase;

import frc.robot.Robot;

public class FollowPoseFromVisionTarget extends SendableCommandBase {

	boolean hadTarget;

	double[] visionData, rangeData;

	public FollowPoseFromVisionTarget() {
		// Use addRequirements() here to declare subsystem dependencies
		// eg. addRequirements(chassis);
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {

		// We first try to aquire a vision target
		// this part doesn't need to run once you've found one
		// for now the robot just sits and checks for a target,
		// in the future make it active and try to find them,
		// based on odometry?

		if (!hadTarget) {
			if (Robot.limelight.getData()[0] == 1) {
				hadTarget = true;
			}
		}

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {}

}
