package frc.robot.commands.auto.actions;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.auto.AutoMotion.HeldPiece;

/**
 * SetIntakeMode sets the wrist to either cargo or hatch 'mode', then either 
 * switches to pointing down to drop/grab cargo or... doesn't
 */
public class SetIntakeMode extends Command {
	HeldPiece intakeType;
	boolean isDown = false;

	public SetIntakeMode(HeldPiece iType) {
		this.intakeType = iType;
		this.isDown = false;
		// requires(Robot.wrist); //TODO this will be a thing that will be uncommented but right now it makes the whole program sad
	}

	public SetIntakeMode(HeldPiece iType, boolean isDown) {
		this.intakeType = iType;
		this.isDown = isDown;
		// requires(Robot.wrist); //TODO this will be a thing that will be uncommented but right now it makes the whole program sad
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		switch (intakeType) {
		case CARGO:
			// TODO set angle to the cargo side
			if (isDown) {
				// TODO change angle to pointing down
			} // doesn't need an else
			break;
		case HATCH:
			// TODO set angle to the hatch side
			break;
		case NONE:
			// This should actually never happen, but if it does it just doesn't do anything
			break;
		}
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		// Don't have to do anything
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return true; //basically instant command
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {}
}
