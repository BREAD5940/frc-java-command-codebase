package frc.robot.commands.auto;

import frc.robot.commands.auto.AutoMotion.GoalHeight;
import frc.robot.commands.auto.AutoMotion.GoalType;
import frc.robot.commands.auto.AutoMotion.HeldPiece;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.subsystems.DriveTrain;

/**
 * Selects and runs an auto command group
 */
public class RunAuto extends AutoCommandGroup {

	public GoalType mGt;
	public GoalHeight mHeight;
	public AutoMotion mMotion;
	public AutoCombo cMotion;
	public String[] cKeys;
	public boolean isDrive;
	public HeldPiece cPiece;
	private boolean begun = false;
	private AutoCommandGroup running;

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
		if (!isDrive) {
			mMotion = new AutoMotion(mHeight, mGt, false);
			running = mMotion.getPrepCommand();
			running.start();
		} else {
			// cMotion = new AutoCombo(cKeys[0], 'L');
			// running=cMotion.getPrepCommand();
			// running.start();
		}
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// Don't need to do anything here
		if (!isDrive && running.done() && !begun) {
			mMotion.getBigCommandGroup().start();
			begun = true;
		}
		// }else if(isDrive&&running.done()&&!begun){
		// 	cMotion.getBigCommandGroup().start();
		// 	begun=true;
		// }
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
