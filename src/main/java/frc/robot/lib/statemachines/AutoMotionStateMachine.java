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
}