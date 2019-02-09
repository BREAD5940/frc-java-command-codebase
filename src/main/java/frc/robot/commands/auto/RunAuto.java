package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.auto.AutoMotion.GoalHeight;
import frc.robot.commands.auto.AutoMotion.GoalType;
import frc.robot.commands.auto.AutoMotion.HeldPiece;
import frc.robot.subsystems.DriveTrain;

/**
 * Selects and runs an auto command group
 */
public class RunAuto extends Command {

	public GoalType mGt;
	public GoalHeight mHeight;
	public AutoMotion mMotion;
	public AutoCombo cMotion;
	public String[] cKeys;
	public boolean isDrive;
	public HeldPiece cPiece;

	public RunAuto(GoalType mGt, GoalHeight mHeight) {
		this.mGt = mGt;
		this.mHeight = mHeight;
		this.isDrive = false;
		// requires(SuperStructure.getInstance());
		requires(DriveTrain.getInstance());
	}

	public RunAuto(HeldPiece cPiece, String... cKeys) {
		this.cKeys = cKeys;
		this.isDrive = true;
		this.cPiece = cPiece;
		// requires(SuperStructure.getInstance());
		requires(DriveTrain.getInstance());
	}

	@Override
	protected void initialize() {
		// Kinda a stupid question but what's the difference between AutoMotion and AutoCombo?
		// is it just that AutoMotion drives straight to a goal, whereas AutoCombo
		if (!isDrive) {
			mMotion = new AutoMotion(mHeight, mGt);
			mMotion.getBigCommandGroup().start();
		} else {
			cMotion = new AutoCombo(cPiece, cKeys);
			cMotion.getBigCommandGroup().start();
		}
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// Don't need to do anything here
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if (!isDrive) {
			return mMotion.getBigCommandGroup().done();
		} else {
			return cMotion.getBigCommandGroup().done();
		}
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {}
}
