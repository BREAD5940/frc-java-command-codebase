package frc.robot.commands.auto;

import frc.robot.commands.auto.actions.DriveDistance;
import frc.robot.commands.auto.actions.SetIntakeMode;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.commands.auto.groups.GrabCargo;
import frc.robot.commands.auto.groups.PickUpHatch;
import frc.robot.commands.auto.groups.PlaceHatch;
import frc.robot.commands.subsystems.drivetrain.FollowVisionTarget;
import frc.robot.subsystems.superstructure.SuperStructure.ElevatorPresets;

/**
 * Creates a command group for a specific automatic motion. Input a type of goal
 * and a height then start the mBigCommandGroup externally In the future, this
 * could change to more inputs depending on the button setup
 * 
 * @author Jocelyn McHugo
 */
public class AutoMotion {

	public enum HeldPiece {
		HATCH, CARGO, NONE
	}

	/**
	 * different heights of goals.
	 * LOW: the lowest level of the rocket and through the hatch of the CARGO ship;
	 * MIDDLE: the middle level of the rocket;
	 * HIGH: the highest level of the rocket;
	 * OVER: dropped into the CARGO ship from above
	 */
	public enum GoalHeight {
		LOW, MIDDLE, HIGH, OVER
	}

	/**
	 * different types of goals on the field.
	 * CARGO_CARGO: put CARGO in the CARGO ship;
	 * ROCKET_CARGO: put CARGO in the rocket;
	 * CARGO_HATCH: put a hatch on the CARGO ship;
	 * ROCKET_HATCH: put a hatch on the rocket;
	 * RETRIEVE_HATCH: pick up a hatch from the loading station;
	 * RETRIEVE_CARGO: pick up CARGO from an unspecified location
	 */
	public enum GoalType {
		CARGO_CARGO, CARGO_HATCH, ROCKET_CARGO, ROCKET_HATCH, RETRIEVE_HATCH, RETRIEVE_CARGO
	}

	private GoalHeight gHeight;
	private GoalType gType;
	private HeldPiece piece;
	private HeldPiece endPiece;
	private AutoCommandGroup mBigCommandGroup;
	private boolean rev;

	/**
	 * generates the command groups based on the inputted goal height/type
	 * @param gHeight
	 *    the height of the goal the robot should aim for (LOW, MIDDLE, HIGH, OVER)
	 * @param gType
	 *    the type of goal
	 */

	public AutoMotion(GoalHeight gHeight, GoalType gType, boolean rev) {
		this.gHeight = gHeight;
		this.gType = gType;
		this.rev = rev;
		//select heldPiece
		if (this.gType == GoalType.CARGO_CARGO || this.gType == GoalType.ROCKET_CARGO) {
			this.piece = HeldPiece.CARGO;
		} else if (this.gType == GoalType.CARGO_HATCH || this.gType == GoalType.ROCKET_HATCH) {
			this.piece = HeldPiece.HATCH;
		} else {
			this.piece = HeldPiece.NONE;
		}

		if (this.piece != HeldPiece.NONE) {
			this.mBigCommandGroup = genPlaceCommands();
		} else {
			this.mBigCommandGroup = genGrabCommands();
		}
	}

	public AutoMotion(boolean isNull) {
		//wheeee this does absolutely NOTING it jusst exists to make AutoCombo UnBork

	}

	/**
	 * Generates commands to pick up a piece based on the parameters of the current AutoMotion
	 * @return
	 *  an ArrayList of commands
	 */
	private AutoCommandGroup genGrabCommands() {
		AutoCommandGroup toReturn = new AutoCommandGroup();
		if (this.gType == GoalType.RETRIEVE_CARGO) {
			// Set the intake to cargo mode
			toReturn.addSequential(new SetIntakeMode(HeldPiece.CARGO, rev));
			// Predefined grab command
			toReturn.addSequential(new GrabCargo());
			this.endPiece = HeldPiece.CARGO;
		} else if (this.gType == GoalType.RETRIEVE_HATCH) {
			// Set the intake to hatch mode
			toReturn.addSequential(new SetIntakeMode(HeldPiece.HATCH, rev));
			// Predefined grab command
			toReturn.addSequential(new PickUpHatch());
			this.endPiece = HeldPiece.HATCH;
		}
		return toReturn;
	}

	/**
	 * @return
	 *  an ArrayList of commands
	 */
	private AutoCommandGroup genPlaceCommands() {
		AutoCommandGroup toReturn = new AutoCommandGroup();

		// Set intake mode
		if (this.gType == GoalType.CARGO_CARGO) {
			toReturn.addSequential(new SetIntakeMode(this.piece, true, rev));
		} else {
			toReturn.addSequential(new SetIntakeMode(this.piece, rev));
		}

		// Align with the vision targets, slightly back from the goal
		//TODO get the robot/limelight 1ft away from the goal

		// Set the elevator to the correct height
		// toReturn.addSequential(new SetElevatorHeight(getElevatorPreset(),false)); //FIXME is there a reason this is commented out?

		if (this.gType == GoalType.CARGO_CARGO) {
			// Drive forward so the intake is over the bay and the bumpers are in the indent thingy
			toReturn.addSequential(new DriveDistance(1+0.2, 20)); // the 0.2 is the bumpers FIXME check distances
		} else {
			// Drive forward so the intake is flush with the port/hatch
			toReturn.addSequential(new DriveDistance(1, 20)); // FIXME check distances
		}

		if (this.piece == HeldPiece.CARGO) {
			// toReturn.addSequential(new AutoIntake(-1, 5)); // TODO change this to something hadled by superstructure?? // do we want the intake on the ss?
		} else if (this.piece == HeldPiece.HATCH) {
			toReturn.addSequential(new PlaceHatch());
		}
		this.endPiece = HeldPiece.NONE;
		return toReturn;

	}

	/**
	 * selects the correct ElevatorPreset from the Elevator subsystems enum based on
	 * the GoalHeight, the GoalType, and the HeldPiece
	 */
	private ElevatorPresets getElevatorPreset() {
		switch (this.gType) {
		case CARGO_CARGO:
			if (this.gHeight == GoalHeight.LOW) {
				return ElevatorPresets.CARGO_SHIP_HATCH;
			} else {
				return ElevatorPresets.CARGO_SHIP_WALL;
			}
		case CARGO_HATCH:
			return ElevatorPresets.CARGO_SHIP_HATCH;
		case ROCKET_CARGO:
			if (this.gHeight == GoalHeight.LOW) {
				return ElevatorPresets.LOW_ROCKET_PORT;
			} else if (gHeight == GoalHeight.MIDDLE) {
				return ElevatorPresets.MIDDLE_ROCKET_PORT;
			} else {
				return ElevatorPresets.HIGH_ROCKET_PORT;
			}
		case ROCKET_HATCH:
			if (this.gHeight == GoalHeight.LOW) {
				return ElevatorPresets.LOW_ROCKET_HATCH;
			} else if (this.gHeight == GoalHeight.MIDDLE) {
				return ElevatorPresets.MIDDLE_ROCKET_HATCH;
			} else {
				return ElevatorPresets.HIGH_ROCKET_HATCH;
			}
		default:
			return ElevatorPresets.CARGO_SHIP_HATCH;
		}
	}

	// id functions

	/**
	 *
	 * @return
	 *  the GoalHeight of the AutoMotion
	 */
	public GoalHeight getGoalHeight() {
		return this.gHeight;
	}

	/**
	 * identification function
	 * @return
	 *  the GoalType of the AutoMotion
	 */
	public GoalType getGoalType() {
		return this.gType;
	}

	/**
	 * identification function
	 * @return
	 *  the HeldPiece of the AutoMotion
	 */
	public HeldPiece getmHeldPiece() {
		return this.piece;
	}

	/**
	 * identification function
	 * @return
	 *  the mBigCommandGroup of the function
	 */
	public AutoCommandGroup getBigCommandGroup() {
		return this.mBigCommandGroup;
	}

	/**
	 * identification function
	 * @return
	 *  the heldpiece at the end of the motion -- for AutoCombo
	 */
	public HeldPiece getEndHeldPiece() {
		return this.endPiece;
	}

}
