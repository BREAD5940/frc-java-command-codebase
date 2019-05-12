// package frc.robot.commands.auto;

// // import frc.robot.lib.statemachines.AutoMotionStateMachine.GoalHeight;
// import frc.robot.lib.statemachines.AutoMotionStateMachine.GoalType;
// import frc.robot.lib.statemachines.AutoMotionStateMachine.HeldPiece;
// import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.superstructure.SuperStructure;

// /**
//  * Selects and runs an auto command group
//  */

// @Deprecated
// public class RunAuto extends AutoCommandGroup {

// 	public GoalType mGt;
// 	public GoalHeight mHeight;
// 	public AutoMotion mMotion;
// 	public AutoCombo cMotion;
// 	public String[] cKeys;
// 	public boolean isDrive;
// 	public HeldPiece cPiece;
// 	private boolean begun = false;
// 	private AutoCommandGroup running;

// 	@Deprecated
// 	public RunAuto(HeldPiece mHP, GoalHeight mHeight) {
// 		switch (mHP) {
// 		case CARGO:
// 			if (mHeight == GoalHeight.LOW) { //FIXME this is dumb
// 				this.mGt = GoalType.CARGO_CARGO;
// 			} else {
// 				this.mGt = GoalType.ROCKET_CARGO;
// 			}
// 			break;
// 		case HATCH:
// 			this.mGt = GoalType.ROCKET_HATCH;
// 			break;
// 		case NONE:
// 			this.mGt = GoalType.RETRIEVE_HATCH; //FIXME this assumes we're never ever using RETRIEVE_CARGO
// 			break;
// 		}

// 		this.mHeight = mHeight;
// 		this.isDrive = false;

// 		addRequirements(SuperStructure.getInstance());
// 		addRequirements(SuperStructure.getInstance().getWrist());
// 		addRequirements(SuperStructure.getInstance().getElbow());
// 		addRequirements(SuperStructure.getElevator());
// 		addRequirements(DriveTrain.getInstance());
// 	}

// 	public RunAuto(GoalType mGt, GoalHeight mHeight) {
// 		this.mGt = mGt;
// 		this.mHeight = mHeight;
// 		this.isDrive = false;
// 		addRequirements(SuperStructure.getInstance());
// 		addRequirements(DriveTrain.getInstance());
// 	}

// 	public RunAuto(HeldPiece cPiece, String... cKeys) {
// 		this.cKeys = cKeys;
// 		this.isDrive = true;
// 		this.cPiece = cPiece;
// 		addRequirements(SuperStructure.getInstance());
// 		addRequirements(DriveTrain.getInstance());
// 	}

// 	@Override
// 	public void initialize() {
// 		if (!isDrive) {
// 			mMotion = new AutoMotion(mHeight, mGt, false);
// 			running = mMotion.getPrepCommand();
// 			running.start();
// 		} else {
// 			// cMotion = new AutoCombo(cKeys[0], 'L');
// 			// running=cMotion.getPrepCommand();
// 			// running.start();
// 		}
// 	}

// 	// Called repeatedly when this Command is scheduled to run
// 	@Override
// 	public void execute() {
// 		// Don't need to do anything here
// 		System.out.println("Done? " + running.done());
// 		if (!isDrive && running.done() && !begun) {
// 			System.out.println("starting big command group");
// 			mMotion.getBigCommandGroup().start();
// 			begun = true;
// 		}
// 		// }else if(isDrive&&running.done()&&!begun){
// 		// 	cMotion.getBigCommandGroup().start();
// 		// 	begun=true;
// 		// }
// 	}

// 	// Make this return true when this Command no longer needs to run execute()
// 	@Override
// 	public boolean isFinished() {
// 		// if (!isDrive) {
// 		// 	return mMotion.getBigCommandGroup().done();
// 		// } else {
// 		// 	return cMotion.getBigCommandGroup().done();
// 		// }
// 		return false;
// 	}

// 	// Called once after isFinished returns true
// 	@Override
// 	protected void end() {}

// 	// Called when another command which requires one or more of the same
// 	// subsystems is scheduled to run
// 	@Override
// 	protected void interrupted() {}
// }
