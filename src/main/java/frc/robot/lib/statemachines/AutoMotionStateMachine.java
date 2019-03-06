package frc.robot.lib.statemachines;

import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class AutoMotionStateMachine extends SendableBase {

	public enum HeldPiece {
		HATCH, CARGO, NONE
	}

	public enum GoalHeight {
		LOW, MIDDLE, HIGH
	}

	public enum GoalType {
		CARGO_CARGO, CARGO_HATCH, ROCKET_CARGO, ROCKET_HATCH, RETRIEVE_HATCH, RETRIEVE_CARGO
	}

	public enum GoalLocation {
		CARGO_SHIP, ROCKET, LOADING, DEPOT
	}

	public enum MotionType {
		PICKUP, PLACE
	}

	private HeldPiece heldPiece = HeldPiece.NONE;
	private HeldPiece goalPiece = HeldPiece.NONE;
	private GoalHeight goalHeight = GoalHeight.LOW;
	private GoalLocation goalLocation = GoalLocation.ROCKET;
	private MotionType motionType = MotionType.PICKUP; //FIXME default pickup or place?
	private GoalType goalType = GoalType.RETRIEVE_HATCH; //FIXME set default

	//Goal Type

	protected void updateGoalType() {
		switch (this.motionType) {
		case PLACE:
			switch (this.goalLocation) {
			case ROCKET:
				switch (this.heldPiece) {
				case HATCH:
					this.goalType = GoalType.ROCKET_HATCH;
				case CARGO:
					this.goalType = GoalType.ROCKET_CARGO;
				case NONE:
					//error, not allowed
					this.goalType = GoalType.ROCKET_HATCH;

				}
			case CARGO_SHIP:
				switch (this.heldPiece) {
				case HATCH:
					this.goalType = GoalType.CARGO_HATCH;
				case CARGO:
					this.goalType = GoalType.CARGO_CARGO;
				case NONE:
					//error, not allowed
					this.goalType = GoalType.CARGO_HATCH;
				}
			}
		case PICKUP:
			switch (this.goalPiece) {
			case HATCH:
				this.goalType = GoalType.RETRIEVE_HATCH;
			case CARGO:
				this.goalType = GoalType.RETRIEVE_CARGO;
			case NONE:
				//error not allowed
				this.goalType = GoalType.RETRIEVE_HATCH;
			}

		}
	}

	protected void updateFromType(){
		switch (this.goalType){
			case CARGO_CARGO:
				this.heldPiece = HeldPiece.CARGO;
				this.goalPiece = HeldPiece.NONE;

				this.goalHeight = GoalHeight.LOW;
				this.goalLocation = GoalLocation.CARGO_SHIP;
				this.motionType = MotionType.PLACE;

			case CARGO_HATCH:
				this.heldPiece = HeldPiece.HATCH;
				this.goalPiece = HeldPiece.NONE;

				this.goalHeight = GoalHeight.LOW;
				this.goalLocation = GoalLocation.CARGO_SHIP;
				this.motionType = MotionType.PLACE;

			case ROCKET_CARGO:
				this.heldPiece = HeldPiece.CARGO;
				this.goalPiece = HeldPiece.NONE;

				//FIXME goal height unknown
				this.goalLocation = GoalLocation.ROCKET;
				this.motionType = MotionType.PLACE;

			case ROCKET_HATCH:
				this.heldPiece = HeldPiece.HATCH;
				this.goalPiece = HeldPiece.NONE;

				//FIXME goal height unknown
				this.goalLocation = GoalLocation.ROCKET;
				this.motionType = MotionType.PLACE;

			case RETRIEVE_CARGO:
				this.heldPiece = HeldPiece.NONE;
				this.goalPiece = HeldPiece.CARGO;

				this.goalHeight = GoalHeight.LOW; //yes?
				this.goalLocation = GoalLocation.DEPOT; //probably
				this.motionType = MotionType.PICKUP;

			case RETRIEVE_HATCH:
				this.heldPiece = HeldPiece.NONE;
				this.goalPiece = HeldPiece.HATCH;

				this.goalHeight = GoalHeight.LOW;
				this.goalLocation = GoalLocation.LOADING;
				this.motionType = MotionType.PICKUP;
		}
	}

	public void setGoalType(GoalType type){
		this.goalType = type;
		updateFromType();
	}

	public GoalType getGoalType() {
		return this.goalType;
	}

	private String goalTypeString() {
		return this.getGoalType().toString();
	}

	//Held Piece

	public void setHeldPiece(HeldPiece piece) {
		this.heldPiece = piece;
		updateGoalType();
	}

	public HeldPiece getHeldPiece() {
		return this.heldPiece;
	}

	private String heldPieceString() {
		return this.getHeldPiece().toString();
	}

	//Goal Piece

	public void setGoalPiece(HeldPiece gp) {
		this.goalPiece = gp;
		updateGoalType();
	}

	public HeldPiece getGoalPiece() {
		return this.goalPiece;
	}

	public String goalPieceString() {
		return this.getGoalPiece().toString();
	}

	//Goal Height

	public void setGoalHeight(GoalHeight gH) {
		this.goalHeight = gH;
		updateGoalType();
	}

	public void goalHeightUp() {
		if (this.goalHeight == GoalHeight.LOW) {
			setGoalHeight(GoalHeight.MIDDLE);
		} else if (this.goalHeight == GoalHeight.MIDDLE) {
			setGoalHeight(GoalHeight.HIGH);
		}
	}

	public void goalHeightDown() {
		if (this.goalHeight == GoalHeight.HIGH) {
			setGoalHeight(GoalHeight.MIDDLE);
		} else if (this.goalHeight == GoalHeight.MIDDLE) {
			setGoalHeight(GoalHeight.LOW);
		}
	}

	public GoalHeight getGoalHeight() {
		return this.goalHeight;
	}

	public String goalHeightString() {
		return this.getGoalHeight().toString();
	}

	//Goal Location

	public void setGoalLocation(GoalLocation loc) {
		this.goalLocation = loc;
		updateGoalType();
	}

	public GoalLocation getGoalLocation() {
		return this.goalLocation;
	}

	public String goalLocationString() {
		return this.getGoalLocation().toString();
	}

	//Motion Type

	public void setMotionType(MotionType type) {
		this.motionType = type;
		updateGoalType();
	}

	public MotionType getMotionType() {
		return this.motionType;
	}

	public String motionTypeString() {
		return this.getMotionType().toString();
	}

	//Sendable

	@Override
	public void initSendable(SendableBuilder builder) {
		builder.setSmartDashboardType("StateMachine"); //FIXME this may or may not make it die
		//So the deal with this method is that it uses a 'named data type', but it keeps it in a heckin' string, and there's not exactly
		//a list of what they are (the ones I've seen are 'subsystem' and 'gyro'), so I have no idea how to do this properly. I think
		//we might want to try using something out of Widget.java, or writing our own. Also we are WAY underutilizing smartdashboard
		//right now, we should probably look more into using more than one of its features

		builder.addStringProperty(".heldPiece", this::heldPieceString, null);
		builder.addStringProperty(".goalPiece", this::goalPieceString, null);
		builder.addStringProperty(".goalHeight", this::goalHeightString, null);
		builder.addStringProperty(".goalLocation", this::goalLocationString, null);
		builder.addStringProperty(".motionType", this::motionTypeString, null);
		builder.addStringProperty(".goalType", this::goalTypeString, null);
	}

}
