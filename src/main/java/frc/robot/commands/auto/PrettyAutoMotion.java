package frc.robot.commands.auto;

import org.ghrobotics.lib.mathematics.units.LengthKt;

import frc.robot.RobotConfig;
import frc.robot.RobotConfig.auto;
import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.subsystems.superstructure.RunIntake;
import frc.robot.commands.subsystems.superstructure.SetHatchMech;
import frc.robot.lib.statemachines.AutoMotionStateMachine;
import frc.robot.lib.statemachines.AutoMotionStateMachine.GoalHeight;
import frc.robot.lib.statemachines.AutoMotionStateMachine.GoalLocation;
import frc.robot.lib.statemachines.AutoMotionStateMachine.HeldPiece;
import frc.robot.lib.statemachines.AutoMotionStateMachine.MainArmPosition;
import frc.robot.lib.statemachines.AutoMotionStateMachine.MotionType;
import frc.robot.planners.SuperstructureMotion;
import frc.robot.states.ElevatorState;
import frc.robot.states.IntakeAngle;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.Intake.HatchMechState;
import frc.robot.subsystems.superstructure.RotatingJoint.RotatingArmState;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

public class PrettyAutoMotion {

	private AutoCommandGroup motionCommands;
	private AutoCommandGroup presetCommands;
	private AutoCommandGroup fullGroup = new AutoCommandGroup();
	private SuperStructureState ssState;

	public PrettyAutoMotion(AutoMotionStateMachine machine) {
		this.ssState = findSuperStructureState(machine);
		this.presetCommands = genPresetMotion(this.ssState);
		this.motionCommands = genMainMotion(machine);
		this.fullGroup.addSequential(this.presetCommands);
		this.fullGroup.addSequential(this.motionCommands);
	}

	private AutoCommandGroup genMainMotion(AutoMotionStateMachine machine) {
		AutoCommandGroup createdGroup = new AutoCommandGroup();
		if (machine.getMotionType() == MotionType.PICKUP) {
			//CHECK if the location is the loading or not
			if (machine.getGoalLocation() == GoalLocation.LOADING) {
				//IF getting a hatch
				if (machine.getGoalPiece() == HeldPiece.HATCH) {
					//CLOSE clamp
					//FIXME is this command still valid?
					createdGroup.addSequential(new SetHatchMech(HatchMechState.kClamped));
					//ALIGN with targets
					// createdGroup.addSequential(new FollowVisionTarget(1, 5, 10));
					//RUN intake
					//RAISE elevator slightly
					//DRIVE back slightly
					//RETURN
				}

				//ELSE
				//RUN the intake
				//RETURN
			} else {
				//RUN the intake for 1 second
				createdGroup.addSequential(new RunIntake(1, 1, 1));
				return createdGroup;
			}

		} else {
			//IF it's a standard placement
			//ALIGN with targets, slightly back
			//IF it's dropped into the cargo
			//ALIGN with the targets, flush
			//RUN the intake in reverse
			//RETURN
		}

		return createdGroup;
	}

	private AutoCommandGroup genPresetMotion(SuperStructureState state) {
		SuperstructureMotion.getInstance().plan(state, SuperStructure.getInstance().lastState); //FIXME lastState is what we want, right?
		return SuperstructureMotion.getInstance().getQueue();
	}

	private SuperStructureState findSuperStructureState(AutoMotionStateMachine machine) {
		ElevatorState eState = new ElevatorState();
		IntakeAngle aState = new IntakeAngle(new RotatingArmState(), new RotatingArmState());

		//FIXME all the fieldPositions need tuning
		switch (machine.getGoalLocation()) {
		case CARGO_SHIP:
			switch (machine.getHeldPiece()) {
			case CARGO:
				eState.setHeight(RobotConfig.auto.fieldPositions.shipWall);
				aState = iPosition.CARGO_DOWN; //FIXME change this angle
				break;
			case HATCH:
				eState.setHeight(RobotConfig.auto.fieldPositions.hatchLowGoal);
				if (machine.getMainArmPosition() == MainArmPosition.BACK) {
					aState = iPosition.HATCH_REVERSE;
				} else {
					aState = iPosition.HATCH; //FIXME is it this or HATCH_PITCHED_UP?
				}
				break;
			case NONE:
				//no
				break;
			}
		case ROCKET:
			switch (machine.getHeldPiece()) {
			case CARGO:
				switch (machine.getMainArmPosition()) {
				case FRONT:
					aState = iPosition.CARGO_PLACE;
					break;
				case IN:
					aState = iPosition.CARGO_PLACE_INSIDE;
					break;
				case BACK:
					aState = iPosition.CARGO_REVERSE;
					break;
				}
				switch (machine.getGoalHeight()) {
				case LOW:
					eState.setHeight(auto.fieldPositions.cargoLowGoal);
					break;
				case MIDDLE:
					eState.setHeight(fieldPositions.cargoMiddleGoal);
					break;
				case HIGH:
					eState.setHeight(fieldPositions.cargoHighGoal);
					break;
				}
				break;
			case HATCH:
				switch (machine.getGoalHeight()) {
				case LOW:
					eState.setHeight(fieldPositions.hatchLowGoal);
					break;
				case MIDDLE:
					eState.setHeight(fieldPositions.hatchMiddleGoal);
					break;
				case HIGH:
					eState.setHeight(fieldPositions.hatchHighGoal);
					break;
				}
				switch (machine.getMainArmPosition()) {
				case FRONT:
					if (machine.getGoalHeight() == GoalHeight.LOW) {
						aState = iPosition.HATCH;
					} else {
						aState = iPosition.HATCH_PITCHED_UP;
					}
					break;
				case IN:
					eState.setHeight(iPosition.HATCH_SLAM_ROCKET_INSIDE_PREP.getElevatorHeight());
					aState = iPosition.HATCH_SLAM_ROCKET_INSIDE_PREP.getAngle();
					break;
				case BACK:
					aState = iPosition.HATCH_REVERSE;
					break;
				}
				break;

			case NONE:
				//no
				break;
			}
		case LOADING:
			switch (machine.getGoalPiece()) {
			case HATCH:
				eState.setHeight(iPosition.HATCH_GRAB_INSIDE_PREP.getElevatorHeight());
				aState = iPosition.HATCH_GRAB_INSIDE_PREP.getAngle();
				break;
			case CARGO:
				//TODO this will do a thing eventually
				break;
			case NONE:
				//no
				break;
			}
			break;
		case DEPOT:
			eState.setHeight(LengthKt.getInch(2));
			aState = iPosition.CARGO_GRAB;
			break;
		}

		return new SuperStructureState(eState, aState);
	}

	/**
	 * Get the full motion -- preset superstructure motion AND automatic motion
	 * @return
	 *    a sequential group of presetCommands and motionCommands
	 */
	public AutoCommandGroup getFullMotion() {
		return this.fullGroup;
	}

	/**
	 * Get the motion to move the superstructure. This can be used in teleop
	 * @return
	 *    the commands to move ONLY the superstructure to its starting pos
	 */
	public AutoCommandGroup getPresetCommands() {
		return this.presetCommands;
	}

	/**
	 * Get the part of the motion that actually does the thing
	 * @return
	 *    the actual moving part
	 */
	public AutoCommandGroup getMotionCommands() {
		return this.motionCommands;
	}
}
