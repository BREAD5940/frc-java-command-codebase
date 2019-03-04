package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.auto.PrettyRunAuto;
import frc.robot.commands.auto.Trajectories;
import frc.robot.commands.auto.routines.PickupHatch;
import frc.robot.commands.subsystems.drivetrain.SetGearCommand;
import frc.robot.commands.subsystems.superstructure.ElevatorMotionMagicTest;
import frc.robot.commands.subsystems.superstructure.SetHatchMech;
import frc.robot.commands.subsystems.superstructure.SuperstructureGoToState;
import frc.robot.lib.DPadButton;
import frc.robot.lib.DPadButton.Direction;
import frc.robot.lib.statemachines.ChangeGoalHeight;
import frc.robot.lib.statemachines.LocationToggle;
import frc.robot.lib.statemachines.PickupToggle;
import frc.robot.lib.statemachines.SetPieceToggle;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.subsystems.Intake.HatchMechState;
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
	// private Joystick driverStation = new Joystick(3); //TODO make a constant

	private Button shift_up_button = new JoystickButton(primaryJoystick, RobotConfig.controls.shift_up_button);
	private Button shift_down_button = new JoystickButton(primaryJoystick, RobotConfig.controls.shift_down_button);
	private Button open_clamp_button = new JoystickButton(secondaryJoystick, xboxmap.Buttons.Y_BUTTON);
	private Button close_clamp_button = new JoystickButton(secondaryJoystick, xboxmap.Buttons.A_BUTTON);

	Button test1Button = new JoystickButton(primaryJoystick, xboxmap.Buttons.Y_BUTTON);
	Button test2Button = new JoystickButton(primaryJoystick, xboxmap.Buttons.A_BUTTON);
	Button test3Button = new JoystickButton(primaryJoystick, xboxmap.Buttons.X_BUTTON);
	Button test4Button = new JoystickButton(primaryJoystick, xboxmap.Buttons.B_BUTTON);
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
	// private Button useMotion = new JoystickButton(secondaryJoystick, xboxmap.Buttons.X_BUTTON);
	//use the full automotion
	private Button useMotion = new DPadButton(secondaryJoystick, Direction.LEFT);
	private Button usePreset = new DPadButton(secondaryJoystick, Direction.RIGHT);

	public OI() {

		toggleHP.toggleWhenPressed(new SetPieceToggle());
		togglePickup.toggleWhenPressed(new PickupToggle());
		toggleGoal.toggleWhenPressed(new LocationToggle());

		goalUp.whenPressed(new ChangeGoalHeight(true));
		goalDown.whenPressed(new ChangeGoalHeight(false));

		useMotion.whenPressed(new PrettyRunAuto(Robot.autoState, false));
		usePreset.whenPressed(new PrettyRunAuto(Robot.autoState, true));

		shift_up_button.whenPressed(new SetGearCommand(Gear.HIGH));
		shift_down_button.whenPressed(new SetGearCommand(Gear.LOW));
		open_clamp_button.whenPressed(new SetHatchMech(HatchMechState.kOpen));
		close_clamp_button.whenPressed(new SetHatchMech(HatchMechState.kClamped));
		if (Trajectories.forward20Feet == null)
			Trajectories.generateAllTrajectories();
		// testAutoButton.whenPressed(DriveTrain.getInstance().followTrajectory(Trajectories.generatedTrajectories.get("test"), true));

		// test2button.whenPressed(new SuperstructureGoToState(
		// 		new SuperStructureState(new ElevatorState(RobotConfig.auto.fieldPositions.hatchLowGoal),
		// 				new IntakeAngle(new RotatingArmState(), new RotatingArmState(Rotation2dKt.getDegree(90)))),
		// 		5));

		// if (plzNoDieElevator.get())
		// 	SuperStructure.getInstance().getCurrentCommand().cancel();
		// if (plzNoDieDriveTrain.get())
		// 	DriveTrain.getInstance().getCurrentCommand().cancel();

		// test1Button.whenPressed(new RunAuto(GoalType.ROCKET_HATCH, GoalHeight.LOW));
		test1Button.whenPressed(new SuperstructureGoToState(iPosition.HATCH_GRAB_INSIDE_PREP)); // y button
		test2Button.whenPressed(new SuperstructureGoToState(iPosition.HATCH_GRAB_INSIDE)); // a button

		// test2Button.whenPressed(new PassThrough()); // a button
		test3Button.whenPressed(new PickupHatch()); // x button
		// test3Button.whenPressed(new FollowVisonTargetTheSecond());
		// test4Button.whenPressed(new PlannerTest(new SuperStructureState(new ElevatorState(LengthKt.getInch(10)), iPosition.HATCH_REVERSE))); // x button
		test4Button.whenPressed(new ElevatorMotionMagicTest());

	}

	public boolean getWaiterButton() {
		return test4Button.get();
	}

	public Joystick getPrimary() {
		return primaryJoystick;
	}

	public double getForwardAxis() {
		return -1 * primaryJoystick.getRawAxis(RobotConfig.controls.forward_axis);
	}

	public double getTurnAxis() {
		return primaryJoystick.getRawAxis(RobotConfig.controls.turn_axis);
	}

	public double getIntakeAxis() {
		return (secondaryJoystick.getRawButton(xboxmap.Buttons.RB_BUTTON)) ? 1 * 1 : 0;
	}

	public double getOuttakeAxis() {
		return (secondaryJoystick.getRawButton(xboxmap.Buttons.LB_BUTTON)) ? 1 * 1 : 0;
	}

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
	public double getIntakeSpeed() {
		return getIntakeAxis() - getOuttakeAxis();
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
