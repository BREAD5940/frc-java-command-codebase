package frc.robot.commands.auto;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import frc.robot.RobotConfig;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.commands.auto.groups.PlaceHatch;
import frc.robot.commands.subsystems.drivetrain.DriveDistance;
import frc.robot.commands.subsystems.drivetrain.FollowVisionTarget;
import frc.robot.commands.subsystems.superstructure.RunIntake;
import frc.robot.commands.subsystems.superstructure.SetHatchMech;
import frc.robot.commands.subsystems.superstructure.SuperstructureGoToState;
import frc.robot.states.ElevatorState;
import frc.robot.states.IntakeAngle;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.Intake.HatchMechState;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

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

	public enum GoalHeight {
		LOW, MIDDLE, HIGH, OVER
	}

	public enum GoalType {
		CARGO_CARGO, CARGO_HATCH, ROCKET_CARGO, ROCKET_HATCH, RETRIEVE_HATCH, RETRIEVE_CARGO
	}

	private GoalHeight gHeight;
	private GoalType gType;
	private HeldPiece piece;
	private HeldPiece endPiece;
	private AutoCommandGroup mBigCommandGroup;
	private AutoCommandGroup mPrepCommand = new AutoCommandGroup();
	private SuperStructureState mSSState;
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
		this.mSSState = new SuperStructureState(new ElevatorState(getElevatorPreset()), getIA());
		this.mPrepCommand.addSequential(new SuperstructureGoToState(this.mSSState));
		if (this.piece != HeldPiece.NONE) {
			this.mBigCommandGroup = genPlaceCommands();
		} else {
			this.mBigCommandGroup = genGrabCommands();
		}
	}

	public AutoMotion(boolean isNull) {
		//This doesn't do anything, but we need it for autocombo
	}

	/**
	 * Generates commands to pick up a piece based on the parameters of the current AutoMotion
	 * @return
	 *  an ArrayList of commands
	 */
	private AutoCommandGroup genGrabCommands() {
		AutoCommandGroup toReturn = new AutoCommandGroup();
		if (this.gType == GoalType.RETRIEVE_CARGO) {
			//TODO target cargo
			toReturn.addSequential(new RunIntake(.75d, 1));
			this.endPiece = HeldPiece.CARGO;
		} else if (this.gType == GoalType.RETRIEVE_HATCH) {
			//TODO align with vision targets
			toReturn.addSequential(new FollowVisionTarget(1, 5, 10));
			//yeet into loading station
			// toReturn.addSequential(new DriveDistance());
			//TODO maybe check the alignment with the center of the hatch with a sensor or some shit?
			//grab
			toReturn.addSequential(new RunIntake(1, 1));
			//pull the hatch out of the brushes
			// toReturn.addSequential(new SuperstructureGoToState(new ElevatorState(getElevatorPreset().plus(LengthKt.getInch(6))))); // lift from brushes
			toReturn.addSequential(new DriveDistance(-0.5)); // back up
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

		// Align with the vision targets, slightly back from the goal
		//TODO get the robot/limelight 1ft away from the goal

		if (this.gType == GoalType.CARGO_CARGO) {
			// Drive forward so the intake is over the bay and the bumpers are in the indent thingy
			toReturn.addSequential(new DriveDistance(0.5 + 0.43)); // the 0.43 is the bumpers FIXME check distances
		} else {
			// Drive forward so the intake is flush with the port/hatch
			toReturn.addSequential(new DriveDistance(0.5)); // FIXME check distances
		}

		if (this.piece == HeldPiece.CARGO) {
			toReturn.addSequential(new RunIntake(-1, 0.5));
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
	private Length getElevatorPreset() {
		switch (this.gHeight) {
		case LOW:
			if (this.gType == GoalType.CARGO_CARGO) {
				return RobotConfig.auto.fieldPositions.shipWall;
			} else if (this.gType == GoalType.ROCKET_CARGO) {
				return RobotConfig.auto.fieldPositions.cargoLowGoal;
			} else {
				return RobotConfig.auto.fieldPositions.hatchLowGoal;
			}
		case MIDDLE:
			if (this.gType == GoalType.ROCKET_CARGO) {
				return RobotConfig.auto.fieldPositions.cargoMiddleGoal;
			} else {
				return RobotConfig.auto.fieldPositions.hatchMiddleGoal;
			}
		case HIGH:
			if (this.gType == GoalType.ROCKET_CARGO) {
				return RobotConfig.auto.fieldPositions.cargoHighGoal;
			} else {
				return RobotConfig.auto.fieldPositions.hatchHighGoal;
			}
		default:
			return RobotConfig.auto.fieldPositions.hatchLowGoal;
		}
	}

	private IntakeAngle getIA() {
		if (this.gType == GoalType.RETRIEVE_CARGO) {
			return iPosition.CARGO_GRAB;
		} else if (this.gType == GoalType.ROCKET_CARGO || this.gType == GoalType.CARGO_CARGO) {
			if (rev) {
				return iPosition.CARGO_REVERSE;
			} else {
				return iPosition.CARGO_PLACE;
			}
		} else if (this.gType == GoalType.CARGO_HATCH || this.gType == GoalType.ROCKET_HATCH || this.gType == GoalType.RETRIEVE_HATCH) {
			if (rev) {
				return iPosition.HATCH_REVERSE;
			} else {
				return iPosition.HATCH;
			}
		}
		return iPosition.CARGO_GRAB;
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
	 * 	the commands for the prep for the motion
	 */
	public AutoCommandGroup getPrepCommand() {
		return this.mPrepCommand;
	}

	/**
	 * identification function
	 * @return
	 *  the heldpiece at the end of the motion -- for AutoCombo
	 */
	public HeldPiece getEndHeldPiece() {
		return this.endPiece;
	}

	public SuperStructureState getSSState() {
		return mSSState;
	}

}
