package frc.robot.commands.auto;

import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;

import frc.robot.commands.auto.AutoMotion.GoalHeight;
import frc.robot.commands.auto.AutoMotion.GoalType;
import frc.robot.commands.auto.AutoMotion.HeldPiece;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;

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
	public AutoCommandGroup mGroup;

	public RunAuto(GoalType mGt, GoalHeight mHeight) {
		this.mGt = mGt;
		this.mHeight = mHeight;
		this.isDrive = false;
		mMotion = new AutoMotion(mHeight, mGt, false);
		mGroup.addSequential(mMotion.getBigCommandGroup());
		// requires(SuperStructure.getInstance());
		requires(DriveTrain.getInstance());
	}

	public RunAuto(HeldPiece cPiece, String... cKeys) {
		this.cKeys = cKeys;
		this.isDrive = true;
		this.cPiece = cPiece;
		cMotion = new AutoCombo(cKeys[0], 'L');
		mGroup.addSequential(cMotion.getBigCommandGroup());
		// requires(SuperStructure.getInstance());
		requires(DriveTrain.getInstance());
	}
	
	public RunAuto(TimedTrajectory<Pose2dWithCurvature> traject, GoalType mGt, GoalHeight mHeight){
		mMotion = new AutoMotion(mHeight, mGt, false);
		mGroup.addParallel(mMotion.getPrepCommand());
		mGroup.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, true));
		mGroup.addSequential(mMotion.getBigCommandGroup());
	}

	@Override
	protected void initialize() {
			mGroup.start();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		// Don't need to do anything here
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return mGroup.done();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {}
}
