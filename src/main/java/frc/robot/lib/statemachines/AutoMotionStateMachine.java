package frc.robot.lib.statemachines;

public class AutoMotionStateMachine {

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
		CARGO_SHIP, ROCKET
	}

	public enum MotionType {
		PICKUP, PLACE
	}

	private HeldPiece heldPiece = HeldPiece.NONE;
	private HeldPiece goalPiece = HeldPiece.NONE;
	private GoalHeight goalHeight = GoalHeight.LOW;
	private GoalLocation goalLocation = GoalLocation.ROCKET;
	private MotionType motionType = MotionType.PICKUP;
	private GoalType goalType = GoalType.RETRIEVE_HATCH; //FIXME set default

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

	public GoalType getGoalType(){
		return this.goalType;
	}

	public void setHeldPiece(HeldPiece piece) {
		this.heldPiece = piece;
		updateGoalType();
	}

	public HeldPiece getHeldPiece() {
		return this.heldPiece;
	}

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

}
