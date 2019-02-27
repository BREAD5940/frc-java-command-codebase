package frc.robot;

import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.auto.Trajectories;
import frc.robot.commands.auto.routines.PickupHatch;
import frc.robot.commands.auto.routines.passthrough.PassThrough;
import frc.robot.commands.subsystems.drivetrain.SetGearCommand;
import frc.robot.commands.subsystems.superstructure.ElevatorMotionMagicTest;
import frc.robot.commands.subsystems.superstructure.PlannerTest;
import frc.robot.commands.subsystems.superstructure.SetHatchMech;
import frc.robot.commands.subsystems.superstructure.SuperstructureGoToState;
import frc.robot.states.ElevatorState;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.subsystems.Intake.HatchMechState;
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
	// private Joystick driverStation = new Joystick(3); //TODO make a constant

	private Button shift_up_button = new JoystickButton(primaryJoystick, RobotConfig.controls.shift_up_button);
	private Button shift_down_button = new JoystickButton(primaryJoystick, RobotConfig.controls.shift_down_button);
	private Button open_clamp_button = new JoystickButton(secondaryJoystick, xboxmap.Buttons.Y_BUTTON);
	private Button close_clamp_button = new JoystickButton(secondaryJoystick, xboxmap.Buttons.A_BUTTON);
	private Button plzNoDieElevator = new JoystickButton(primaryJoystick, xboxmap.Buttons.LEFT_START_BUTTON);
	private Button plzNoDieDriveTrain = new JoystickButton(primaryJoystick, xboxmap.Buttons.RIGHT_START_BUTTON);
	Button secondaryTest1 = new JoystickButton(secondaryJoystick, xboxmap.Buttons.B_BUTTON);
	Button secondaryTest2 = new JoystickButton(secondaryJoystick, xboxmap.Buttons.X_BUTTON);
	// Button autobutton3 = new JoystickButton(primaryJoystick, xboxmap.Buttons.Y_BUTTON);

	// TODO change these to a button console once created
	// Button auto_place_cargo_cargo_button = new JoystickButton(secondaryJoystick, xboxmap.Buttons.X_BUTTON);
	// Button auto_place_hatch_cargo_button = new JoystickButton(secondaryJoystick, xboxmap.Buttons.B_BUTTON);
	Button test1Button = new JoystickButton(primaryJoystick, xboxmap.Buttons.Y_BUTTON);
	Button test2Button = new JoystickButton(primaryJoystick, xboxmap.Buttons.A_BUTTON);
	Button test3Button = new JoystickButton(primaryJoystick, xboxmap.Buttons.X_BUTTON);
	Button test4Button = new JoystickButton(primaryJoystick, xboxmap.Buttons.B_BUTTON);
	// Button test2button = new JoystickButton(secondaryJoystick, xboxmap.Buttons.X_BUTTON);

	// File file = new File("/home/lvuser/deploy/paths/test.pf1.csv");
	// Trajectory trajectory = Pathfinder.readFromCSV(file);
	// PathfinderTrajectory pftraj = /*new PathfinderTrajectory(trajectory);*/ PathfinderTrajectory.readFromTrajectory(trajectory);

	// Button cargoOverButton = new JoystickButton(driverStation, DriverstationMap.Buttons.cargoCargo);
	// Button cargo1Button = new JoystickButton(driverStation, DriverstationMap.Buttons.cargo1);
	// Button cargo2Button = new JoystickButton(driverStation, DriverstationMap.Buttons.cargo2);
	// Button cargo3Button = new JoystickButton(driverStation, DriverstationMap.Buttons.cargo3);

	// Button hatch1Button = new JoystickButton(driverStation, DriverstationMap.Buttons.hatch1);
	// Button hatch2Button = new JoystickButton(driverStation, DriverstationMap.Buttons.hatch2);
	// Button hatch3Button = new JoystickButton(driverStation, DriverstationMap.Buttons.hatch3);

	// Button hatchPickupButton = new JoystickButton(driverStation, DriverstationMap.Buttons.hatchPickup);
	// Button intakeButton = new JoystickButton(driverStation, DriverstationMap.Buttons.intake);
	// Button outtakeButton = new JoystickButton(driverStation, DriverstationMap.Buttons.outtake);

	public OI() {
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

		if (plzNoDieElevator.get())
			SuperStructure.getInstance().getCurrentCommand().cancel();
		if (plzNoDieDriveTrain.get())
			DriveTrain.getInstance().getCurrentCommand().cancel();

		// test1Button.whenPressed(new RunAuto(GoalType.ROCKET_HATCH, GoalHeight.LOW));
		test1Button.whenPressed(new SuperstructureGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH)); // y button
		test2Button.whenPressed(new PassThrough()); // a button
		test3Button.whenPressed(new PickupHatch()); // x button
		// test3Button.whenPressed(new FollowVisonTargetTheSecond());
		// test4Button.whenPressed(new PlannerTest(new SuperStructureState(new ElevatorState(LengthKt.getInch(10)), iPosition.HATCH_REVERSE))); // x button
		test4Button.whenPressed(new ElevatorMotionMagicTest()); // b button
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
