package frc.robot;

import java.util.Arrays;
import java.util.List;

import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.auto.InstantRunnable;
import frc.robot.commands.auto.routines.LoadingToRocketF;
import frc.robot.commands.auto.routines.TwoHatchOneCargo;
import frc.robot.commands.subsystems.drivetrain.SetGearCommand;
import frc.robot.commands.subsystems.superstructure.SetHatchMech;
import frc.robot.commands.subsystems.superstructure.SuperstructureGoToState;
import frc.robot.commands.subsystems.superstructure.ToggleClamp;
import frc.robot.lib.DPadButton;
import frc.robot.lib.motion.Util;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.lib.obj.factories.SequentialCommandFactory;
import frc.robot.planners.SuperstructureMotion;
import frc.robot.states.ElevatorState;
import frc.robot.states.IntakeAngle;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.subsystems.Intake.HatchMechState;
import frc.robot.subsystems.superstructure.RotatingJoint.RotatingArmState;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

/**
 * Operator Input not Out-In
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 * For use with commands and stuff.
 *
 * @author Matthew Morley
 */
public class OI {

	private Joystick primaryJoystick = new Joystick(RobotConfig.controls.primary_joystick_port);
	private Joystick secondaryJoystick = new Joystick(RobotConfig.controls.secondary_joystick_port);
	private Joystick driverStation = new Joystick(5); //TODO make a constant

	private Button shift_up_button = new JoystickButton(primaryJoystick, RobotConfig.controls.shift_up_button);
	private Button shift_down_button = new JoystickButton(primaryJoystick, RobotConfig.controls.shift_down_button);
	private Button open_clamp_button = new JoystickButton(secondaryJoystick, xboxmap.Buttons.Y_BUTTON);
	private Button close_clamp_button = new JoystickButton(secondaryJoystick, xboxmap.Buttons.A_BUTTON);

	// TODO change these to a button console once created

	Button primaryYButton = new JoystickButton(primaryJoystick, xboxmap.Buttons.Y_BUTTON);
	Button primaryAButton = new JoystickButton(primaryJoystick, xboxmap.Buttons.A_BUTTON);
	Button primaryXButton = new JoystickButton(primaryJoystick, xboxmap.Buttons.X_BUTTON);
	Button primaryBButton = new JoystickButton(primaryJoystick, xboxmap.Buttons.B_BUTTON);

	Button secondaryDpadUp = new DPadButton(secondaryJoystick, DPadButton.Direction.UP);
	Button secondaryDpadDown = new DPadButton(secondaryJoystick, DPadButton.Direction.DOWN);
	Button secondaryDpadLeft = new DPadButton(secondaryJoystick, DPadButton.Direction.LEFT);
	Button secondaryDpadRight = new DPadButton(secondaryJoystick, DPadButton.Direction.RIGHT);

	Button primaryDpadUp = new DPadButton(primaryJoystick, DPadButton.Direction.UP);
	Button primaryDpadDown = new DPadButton(primaryJoystick, DPadButton.Direction.DOWN);
	Button primaryDpadLeft = new DPadButton(primaryJoystick, DPadButton.Direction.LEFT);
	Button primaryDpadRight = new DPadButton(primaryJoystick, DPadButton.Direction.RIGHT);

	Button primaryRightStart = new JoystickButton(primaryJoystick, xboxmap.Buttons.RIGHT_START_BUTTON);
	Button primaryLeftStart = new JoystickButton(primaryJoystick, xboxmap.Buttons.LEFT_START_BUTTON);

	Button dsCargo1 = new JoystickButton(driverStation, 9);
	Button dsCargo2 = new JoystickButton(driverStation, 6);
	Button dsCargo3 = new JoystickButton(driverStation, 10);
	Button dsCargoCargo = new JoystickButton(driverStation, 7);
	Button dsCargoIn = new JoystickButton(driverStation, 2);

	Button dsHatch1 = new JoystickButton(driverStation, 12);
	Button dsHatch2 = new JoystickButton(driverStation, 1);
	Button dsHatch3 = new JoystickButton(driverStation, 11);
	Button dsHatchIn = new JoystickButton(driverStation, 3);

	Button dsClampToggle = new JoystickButton(driverStation, 8);

	// Button test2button = new JoystickButton(secondaryJoystick, xboxmap.Buttons.X_BUTTON);

	// Button test1Button = new JoystickButton(primaryJoystick, xboxmap.Buttons.Y_BUTTON);
	// Button test2Button = new JoystickButton(primaryJoystick, xboxmap.Buttons.A_BUTTON);
	// Button test3Button = new JoystickButton(primaryJoystick, xboxmap.Buttons.X_BUTTON);
	// Button test4Button = new JoystickButton(primaryJoystick, xboxmap.Buttons.B_BUTTON);
	// private Button plzNoDieElevator = new JoystickButton(primaryJoystick, xboxmap.Buttons.LEFT_START_BUTTON);
	// private Button plzNoDieDriveTrain = new JoystickButton(primaryJoystick, xboxmap.Buttons.RIGHT_START_BUTTON);
	// Button secondaryTest1 = new JoystickButton(secondaryJoystick, xboxmap.Buttons.B_BUTTON);
	// Button secondaryTest2 = new JoystickButton(secondaryJoystick, xboxmap.Buttons.X_BUTTON);

	//TODO is the location for each button logical?

	//toggle which heldpiece we have
	private Button toggleHP = new JoystickButton(secondaryJoystick, xboxmap.Buttons.B_BUTTON);
	//aim for the next highest goal
	private Button goalUp = new DPadButton(secondaryJoystick, DPadButton.Direction.UP);
	//aim for the next lowest goal
	private Button goalDown = new DPadButton(secondaryJoystick, DPadButton.Direction.DOWN);
	//pickup vs placement
	private Button togglePickup = new JoystickButton(secondaryJoystick, xboxmap.Buttons.Y_BUTTON);
	//cargo ship vs rocket
	private Button toggleGoal = new JoystickButton(secondaryJoystick, xboxmap.Buttons.A_BUTTON);
	//actually use the automotion vs just presets
	// private Button secondaryDpadLeft = new JoystickButton(secondaryJoystick, xboxmap.Buttons.X_BUTTON);
	//use the full automotion
	// private Button secondaryDpadLeft = new DPadButton(secondaryJoystick, Direction.LEFT);
	// private Button usePreset = new DPadButton(secondaryJoystick, Direction.RIGHT);

	public OI() {

		// toggleHP.toggleWhenPressed(new SetPieceToggle());
		// togglePickup.toggleWhenPressed(new PickupToggle());
		// toggleGoal.toggleWhenPressed(new LocationToggle());

		// goalUp.whenPressed(new ChangeGoalHeight(true));
		// goalDown.whenPressed(new ChangeGoalHeight(false));

		// useMotion.whenPressed(new PrettyRunAuto(Robot.autoState, false));
		// usePreset.whenPressed(new PrettyRunAuto(Robot.autoState, true));

		shift_up_button.whenPressed(new SetGearCommand(Gear.HIGH));
		shift_down_button.whenPressed(new SetGearCommand(Gear.LOW));
		open_clamp_button.whenPressed(new SetHatchMech(HatchMechState.kOpen)); // y button
		close_clamp_button.whenPressed(new SetHatchMech(HatchMechState.kClamped)); // a button
		dsClampToggle.whenPressed(new ToggleClamp());

		// primaryDpadUp.whenPressed(new PickupHatch());
		primaryDpadDown.whenPressed(new SuperstructureGoToState(
				new ElevatorState(LengthKt.getInch(26)),
				new IntakeAngle(
						new RotatingArmState(RoundRotation2d.getDegree(-94)),
						new RotatingArmState(RoundRotation2d.getDegree(-42)))));

		SuperStructureState Start1 = new SuperStructureState(new ElevatorState(LengthKt.getInch(5)),
				new IntakeAngle(new RotatingArmState(RoundRotation2d.getDegree(0)), new RotatingArmState(RoundRotation2d.getDegree(0))));

		SuperStructureState Goal1 = new SuperStructureState(new ElevatorState(LengthKt.getInch(10)),
				new IntakeAngle(new RotatingArmState(RoundRotation2d.getDegree(0)), new RotatingArmState(RoundRotation2d.getDegree(0))));

		// var planned = SuperstructureMotion.getInstance().plan(
		// 	iPosition.HATCH_GRAB_INSIDE, new SuperStructureState(
		// 		new ElevatorState(LengthKt.getInch(2)), iPosition.CARGO_GRAB
		// 	)
		// 	);

		primaryBButton.whenPressed(new SuperstructureMotion(iPosition.HATCH_GRAB_INSIDE));

		// primaryDpadUp.whenPressed(new FollowVisionTargetTheSecond(4.3));
		// primaryDpadUp.whenPressed(new DriveDistanceToVisionTarget(LengthKt.getInch(40), 6));

		// primaryAButton.whileHeld(new HybridDriverAssist(7));
		// primaryBButton.whenPressed(new SuperstructureMotion(new SuperStructureState(new ElevatorState(LengthKt.getInch(2)), iPosition.CARGO_GRAB), iPosition.HATCH_GRAB_INSIDE_PREP));
		// primaryXButton.whenPressed()

		// cargo presets
		dsCargoIn.whenPressed(SequentialCommandFactory.getSequentialCommands(
				Arrays.asList(
						new SetHatchMech(HatchMechState.kOpen),
						new SuperstructureGoToState(LengthKt.getInch(5), SuperStructure.iPosition.CARGO_GRAB))));

		dsCargo1.whenPressed(SequentialCommandFactory.getSequentialCommands(
				Arrays.asList(
						new SetHatchMech(HatchMechState.kClamped),
						new SuperstructureGoToState(fieldPositions.cargoLowGoal, SuperStructure.iPosition.CARGO_PLACE))));

		dsCargo2.whenPressed(SequentialCommandFactory.getSequentialCommands(
				Arrays.asList(
						new SetHatchMech(HatchMechState.kClamped),
						new SuperstructureGoToState(fieldPositions.cargoMiddleGoal, SuperStructure.iPosition.CARGO_PLACE))));

		dsCargo3.whenPressed(SequentialCommandFactory.getSequentialCommands(
				Arrays.asList(
						new SetHatchMech(HatchMechState.kClamped),
						new SuperstructureGoToState(fieldPositions.cargoHighGoal, SuperStructure.iPosition.CARGO_PLACE))));

		// secondaryDpadRight.whenPressed(new SuperstructureGoToState(fieldPositions.cargoMiddleGoal, SuperStructure.iPosition.CARGO_PLACE));
		// secondaryDpadUp.whenPressed(new SuperstructureGoToState(fieldPositions.cargoHighGoal, SuperStructure.iPosition.CARGO_PLACE));

		// hatch presets
		// dsHatchIn.whenPressed(new SuperstructureGoToState(iPosition.HATCH_GRAB_INSIDE));
		dsHatch1.whenPressed(new SuperstructureGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));
		dsHatch2.whenPressed(new SuperstructureGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH));
		dsHatch3.whenPressed(new SuperstructureGoToState(fieldPositions.hatchHighGoal, iPosition.HATCH));

		// primaryDpadRight.whenPressed(new SuperstructureGoToState(iPosition.HATCH_GRAB_INSIDE));
		// primaryDpadUp.whenPressed(new SuperstructureGoToState(iPosition.HATCH_GRAB_INSIDE));

		primaryDpadUp.whenPressed(new TwoHatchOneCargo());

		secondaryDpadUp.whenPressed(new InstantRunnable(() -> {
			System.out.println(SuperStructure.getInstance().getWrist().getMaster().getSensorCollection().getPulseWidthPosition());
		}));

		primaryRightStart.whenPressed(new LoadingToRocketF('R', 'R'));

		// test3Button.whenPressed(new FollowVisonTargetTheSecond());\
		// test4Button.whenPressed(new PlannerTest(new SuperStructureState(new ElevatorState(LengthKt.getInch(10)), iPosition.HATCH_REVERSE))); // x button
		// primaryBButton.whenPressed(new visionTest()); // b button
		// test4Button.whenPressed(new RunAuto(GoalType.RETRIEVE_HATCH, GoalHeight.LOW)); // b button - used now for dealy

		// cargoOverButton.whenPressed(new RunAuto(HeldPiece.CARGO, GoalHeight.OVER));
		// cargo1Button.whenPressed(new RunAuto(HeldPiece.CARGO, GoalHeight.LOW));
		// cargo2Button.whenPressed(new RunAuto(HeldPiece.CARGO, GoalHeight.MIDDLE));
		// cargo3Button.whenPressed(new RunAuto(HeldPiece.CARGO, GoalHeight.HIGH));

		// hatch1Button.whenPressed(new RunAuto(HeldPiece.HATCH, GoalHeight.LOW));
		// hatch2Button.whenPressed(new RunAuto(HeldPiece.HATCH, GoalHeight.MIDDLE));
		// hatch3Button.whenPressed(new RunAuto(HeldPiece.HATCH, GoalHeight.HIGH));

		// hatchPickupButton.whenPressed(new RunAuto(GoalType.RETRIEVE_HATCH, GoalHeight.LOW));
		// intakeButton.whileHeld(new RunIntake(1,5));
		// outtakeButton.whileHeld(new RunIntake(-1,5));

		// yeetInACircleButton.whenPressed(DriveTrain.getInstance().followTrajectory(Trajectories.forward20Feet, true));
		// yeetInACircleButton.whenPressed(new TurnInPlace(Rotation2dKt.getDegree(180), false));

		// TODO why does this throw a null pointer
		// auto_place_cargo_cargo_button.whenPressed(new RunAuto(mGoalType.CARGO_CARGO, Robot.mGh.getSelected()));
		// auto_place_hatch_cargo_button.whenPressed(new RunAuto(mGoalType.CARGO_HATCH, Robot.mGh.getSelected()));
		// auto_place_cargo_rocket_button.whenPressed(new RunAuto(mGoalType.ROCKET_CARGO, Robot.mGh.getSelected()));
		// auto_place_hatch_rocket_button.whenPressed(new RunAuto(mGoalType.ROCKET_HATCH, Robot.mGh.getSelected()));
		// auto_grab_hatch_button.whenPressed(new RunAuto(mGoalType.RETRIEVE_HATCH, Robot.mGh.getSelected()));
		// auto_grab_cargo_button.whenPressed(new RunAuto(mGoalType.RETRIEVE_CARGO, Robot.mGh.getSelected()));

		// auto_grab_cargo_button.whenPressed(new SetElevatorHeight(9.316859344380049 - 1.5 ) );
		// auto_grab_hatch_button.whenPressed(new SetElevatorHeight(30));
		// auto_place_hatch_cargo_button.whenPressed(new RunAuto(mGoalType.CARGO_HATCH, AutoMotion.mGoalHeight.LOW));
		// auto_place_cargo_rocket_button.whenPressed(new RunAuto(mGoalType.ROCKET_CARGO, AutoMotion.mGoalHeight.LOW));
		// auto_place_hatch_rocket_button.whenPressed(new RunAuto(mGoalType.ROCKET_HATCH, AutoMotion.mGoalHeight.LOW));
		// auto_grab_hatch_button.whenPressed(new RunAuto(mGoalType.RETRIEVE_HATCH, AutoMotion.mGoalHeight.LOW));
		// auto_grab_cargo_button.whenPressed(new RunAuto(mGoalType.RETRIEVE_CARGO, AutoMotion.mGoalHeight.LOW));
		// turnAutoButton.whenPressed(new PurePursuitPathCommand());
		// autobutton2.whenPressed(new RamsetePathFollower("filePath"));
		// autobutton3.whenPressed(new DriveTrajectoryPathfinder("mFile"));
		// open_clamp_button.whenPressed(new OpenClamp());
		// close_clamp_button.whenPressed(new CloseClamp());
		// auto_place_cargo_cargo_button.whenPressed(new SetElevatorHeight(9.316859344380049 - 1.5 ) );
		// auto_place_hatch_cargo_button.whenPressed(new SetElevatorHeight(30));

		// auto_place_cargo_rocket_button.whenPressed(new visionTest());
		// auto_place_hatch_rocket_button.whenPressed(new RunDriveMotionPlanner(  pftraj ) );

		// auto_place_hatch_cargo_button.whenPressed(new RunAuto(mGoalType.CARGO_HATCH, AutoMotion.mGoalHeight.LOW));
		// auto_place_cargo_rocket_button.whenPressed(new RunAuto(mGoalType.ROCKET_CARGO, AutoMotion.mGoalHeight.LOW));
		// auto_place_hatch_rocket_button.whenPressed(new RunAuto(mGoalType.ROCKET_HATCH, AutoMotion.mGoalHeight.LOW));
		// auto_grab_hatch_button.whenPressed(new RunAuto(mGoalType.RETRIEVE_HATCH, AutoMotion.mGoalHeight.LOW));
		// auto_grab_cargo_button.whenPressed(new RunAuto(mGoalType.RETRIEVE_CARGO, AutoMotion.mGoalHeight.LOW));
		// turnAutoButton.whenPressed(new PurePursuitPathCommand());
		// autobutton2.whenPressed(new RamsetePathFollower("filePath"));
		// autobutton3.whenPressed(new ForwardFiveMeters()  );
		// open_clamp_button.whenPressed(new OpenClamp());
		// close_clamp_button.whenPressed(new CloseClamp());
		// open_clamp_button.whenPressed(new PurePursuit());
	}

	public boolean getWaiterButton() {
		return primaryBButton.get();
	}

	public Joystick getPrimary() {
		return primaryJoystick;
	}

	public Joystick getSecondary() {
		return secondaryJoystick;
	}

	public List<Joystick> getAllSticks() {
		return Arrays.asList(getPrimary(), getSecondary());
	}

	public enum OperatorControllers {
		PRIMARY, SECONDARY;
	}

	public void setRumble(RumbleType side, Joystick stick, double value) {
		value = Util.limit(value, 0, 1);
		stick.setRumble(side, value);
	}

	public void setAllRumble(double value) {
		setRumble(RumbleType.kLeftRumble, getPrimary(), value);
		setRumble(RumbleType.kLeftRumble, getSecondary(), value);

		setRumble(RumbleType.kRightRumble, getPrimary(), value);
		setRumble(RumbleType.kRightRumble, getSecondary(), value);
	}

	public double getForwardAxis() {
		return -1 * primaryJoystick.getRawAxis(RobotConfig.controls.forward_axis);
	}

	public double getTurnAxis() {
		return primaryJoystick.getRawAxis(RobotConfig.controls.turn_axis);
	}

	// public double getIntakeAxis() {
	// 	return (secondaryJoystick.getRawButton(xboxmap.Buttons.RB_BUTTON)) ? 1 * 1 : 0;
	// }

	// public double getCargoOuttake() {
	// 	return (secondaryJoystick.getRawButton(xboxmap.Buttons.X_BUTTON)) ? 1 * 1 : 0;
	// }

	// public double getCargoIntake() {
	// 	return (secondaryJoystick.getRawButton(xboxmap.Buttons.B_BUTTON)) ? 1 * 1 : 0;
	// }

	// public double getOuttakeAxis() {
	// 	return (secondaryJoystick.getRawButton(xboxmap.Buttons.LB_BUTTON)) ? 1 * 1 : 0;
	// }

	public double getWristAxis() {
		return secondaryJoystick.getRawAxis(5);
	}

	public double getElbowAxis() {
		return (secondaryJoystick.getRawAxis(2) - secondaryJoystick.getRawAxis(3)) * -1; // triggers
	}

	public double getDSElbowAxis() {
		return 0;//(driverStation.getRawAxis(DriverstationMap.Axes.elbowStick));
	}

	public double getDSElevatorAxis() {
		return 0;//(driverStation.getRawAxis(DriverstationMap.Axes.elevatorStick));
	}

	/**
	 * Get intake speed is the difference between intake and outtake axis speeds
	 */
	public double getHatchSpeed() {
		// System.out.println("HATCH SPEED: " + driverStation.getRawAxis(0));
		return driverStation.getRawAxis(0);
	}

	public double getCargoSpeed() {
		return driverStation.getRawAxis(1) * -1;
	}

	public double getElevatorAxis() {
		// return 0;
		return secondaryJoystick.getRawAxis(RobotConfig.controls.xbox_elevator_axis) * -1;
	}

	public double getThrottleAxis() {
		return 0;
	}//secondaryJoystick.getRawAxis(RobotConfig.controls.throttle_elevator_axis); }

	// There are a few additional built in buttons you can use. Additionally,
	// by subclassing Button you can create custom triggers and bind those to
	// commands the same as any other Button.

	//// TRIGGERING COMMANDS WITH BUTTONS
	// Once you have a button, it's trivial to bind it to a button in one of
	// three ways:

	// Start the command when the button is pressed and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenPressed(new ExampleCommand());

	// Run the command while the button is being held down and interrupt it once
	// the button is released.
	// button.whileHeld(new ExampleCommand());

	// Start the command when the button is released and let it run the command
	// until it is finished as determined by it's isFinished method.
	// button.whenReleased(new ExampleCommand());
}
