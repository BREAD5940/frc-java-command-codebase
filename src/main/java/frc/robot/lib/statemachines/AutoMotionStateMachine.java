package frc.robot.lib.statemachines;

public class AutoMotionStateMachine{

  public enum HeldPiece {
		HATCH, CARGO, NONE
	}

	public enum GoalHeight {
		LOW, MIDDLE, HIGH, OVER
	}

	public enum GoalType {
		CARGO_CARGO, CARGO_HATCH, ROCKET_CARGO, ROCKET_HATCH, RETRIEVE_HATCH, RETRIEVE_CARGO
	}

	public enum GoalLocation{
		CARGO_SHIP, ROCKET
	}

	public enum MotionType{
		PICKUP, PLACE
	}

	private static HeldPiece heldPiece = HeldPiece.NONE;
	private static HeldPiece goalPiece = HeldPiece.NONE;
	private static GoalHeight goalHeight = GoalHeight.LOW;
	private static GoalLocation goalLocation = GoalLocation.ROCKET;
	private static MotionType motionType = MotionType.PICKUP;
	private static GoalType goalType = GoalType.RETRIEVE_HATCH; //FIXME set default

	protected static void updateGoalType(){
		switch (motionType){
			case PLACE:
				switch (goalLocation){
					case ROCKET:
						switch (heldPiece){
							case HATCH:
								goalType = GoalType.ROCKET_HATCH;
							case CARGO:
								goalType = GoalType.ROCKET_CARGO;
							case NONE:
								//error, not allowed
								goalType = GoalType.ROCKET_HATCH;
								
						}
					case CARGO_SHIP:
						switch(heldPiece){
							case HATCH:
								goalType = GoalType.CARGO_HATCH;
							case CARGO:
								goalType = GoalType.CARGO_CARGO;
							case NONE:
								//error, not allowed
								goalType = GoalType.CARGO_HATCH;
						}
				}
			case PICKUP:
				switch (goalPiece){
					case HATCH:
						goalType = GoalType.RETRIEVE_HATCH;
					case CARGO:
						goalType = GoalType.RETRIEVE_CARGO;
					case NONE:
						//error not allowed
						goalType = GoalType.RETRIEVE_HATCH;
				}

		}
	}


	public static void setHeldPiece(HeldPiece piece){
		heldPiece = piece;
		updateGoalType();
	}

	public static HeldPiece getHeldPiece(){
		return heldPiece;
	}

	public static void setGoalHeight(GoalHeight gH){
		goalHeight = gH;
		updateGoalType();
	}

	public static GoalHeight getGoalHeight(){
		return goalHeight;
	}

	
}