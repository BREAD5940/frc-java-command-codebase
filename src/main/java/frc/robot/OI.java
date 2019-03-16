package frc.robot;

import java.util.Arrays;

import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.KillAuto;
import frc.robot.commands.subsystems.drivetrain.HybridDriverAssist;
import frc.robot.commands.subsystems.drivetrain.SetGearCommand;
import frc.robot.commands.subsystems.superstructure.ArmMove;
import frc.robot.commands.subsystems.superstructure.ElevatorMove;
import frc.robot.commands.subsystems.superstructure.JankyGoToState;
import frc.robot.commands.subsystems.superstructure.SetElevatorGear;
import frc.robot.commands.subsystems.superstructure.SetHatchMech;
import frc.robot.lib.AnalogButton;
import frc.robot.lib.DPadButton;
import frc.robot.lib.motion.Util;
import frc.robot.lib.obj.factories.SequentialCommandFactory;
import frc.robot.states.ElevatorState;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.subsystems.Intake.HatchMechState;
import frc.robot.subsystems.superstructure.Elevator.ElevatorGear;
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
	// private Joystick secondaryJoystick = new Joystick(RobotConfig.controls.secondary_joystick_port);
	private Joystick driverStation = new Joystick(5); //TODO make a constant

	private Button shift_up_button = new JoystickButton(primaryJoystick, RobotConfig.controls.shift_up_button);
	private Button shift_down_button = new JoystickButton(primaryJoystick, RobotConfig.controls.shift_down_button);
	// private Button open_clamp_button = new JoystickButton(secondaryJoystick, xboxmap.Buttons.Y_BUTTON);
	// private Button close_clamp_button = new JoystickButton(secondaryJoystick, xboxmap.Buttons.A_BUTTON);

	// TODO change these to a button console once created

	Button primaryYButton = new JoystickButton(primaryJoystick, xboxmap.Buttons.Y_BUTTON);
	Button primaryAButton = new JoystickButton(primaryJoystick, xboxmap.Buttons.A_BUTTON);
	Button primaryXButton = new JoystickButton(primaryJoystick, xboxmap.Buttons.X_BUTTON);
	Button primaryBButton = new JoystickButton(primaryJoystick, xboxmap.Buttons.B_BUTTON);

	// Button secondaryDpadUp = new DPadButton(secondaryJoystick, DPadButton.Direction.UP);
	// Button secondaryDpadDown = new DPadButton(secondaryJoystick, DPadButton.Direction.DOWN);
	// Button secondaryDpadLeft = new DPadButton(secondaryJoystick, DPadButton.Direction.LEFT);
	// Button secondaryDpadRight = new DPadButton(secondaryJoystick, DPadButton.Direction.RIGHT);

	public Button primaryDpadUp = new DPadButton(primaryJoystick, DPadButton.Direction.UP);
	Button primaryDpadDown = new DPadButton(primaryJoystick, DPadButton.Direction.DOWN);
	Button primaryDpadLeft = new DPadButton(primaryJoystick, DPadButton.Direction.LEFT);
	Button primaryDpadRight = new DPadButton(primaryJoystick, DPadButton.Direction.RIGHT);

	Button primaryRightStart = new JoystickButton(primaryJoystick, xboxmap.Buttons.RIGHT_START_BUTTON);
	Button primaryLeftStart = new JoystickButton(primaryJoystick, xboxmap.Buttons.LEFT_START_BUTTON);

	Button primaryRightAnalogButton = new AnalogButton(primaryJoystick, xboxmap.Axis.RIGHT_TRIGGER, .8);

	Button dsCargo1 = new JoystickButton(driverStation, 7);
	Button dsCargo2 = new JoystickButton(driverStation, 6);
	Button dsCargo3 = new JoystickButton(driverStation, 5);
	Button dsCargoShip = new JoystickButton(driverStation, 8);
	Button dsCargoIn = new JoystickButton(driverStation, 12);

	Button dsHatch1 = new JoystickButton(driverStation, 3);
	Button dsHatch2 = new JoystickButton(driverStation, 2);
	Button dsHatch3 = new JoystickButton(driverStation, 1);
	Button dsHatchIn = new JoystickButton(driverStation, 10);

	Button dsJogUp = new JoystickButton(driverStation, 9);
	Button dsJogDown = new JoystickButton(driverStation, 11);

	public OI() {

		shift_up_button.whenPressed(new SetGearCommand(Gear.HIGH));
		shift_down_button.whenPressed(new SetGearCommand(Gear.LOW));

		// cargo presets
		dsCargoIn.whenPressed(SequentialCommandFactory.getSequentialCommands(
				Arrays.asList(
						new SetHatchMech(HatchMechState.kOpen),
						new JankyGoToState(LengthKt.getInch(5.5), SuperStructure.iPosition.CARGO_GRAB))));

		dsCargo1.whenPressed(SequentialCommandFactory.getSequentialCommands(
				Arrays.asList(
						new SetHatchMech(HatchMechState.kClamped),
						new JankyGoToState(fieldPositions.cargoLowGoal, SuperStructure.iPosition.CARGO_PLACE))));

		dsCargo2.whenPressed(SequentialCommandFactory.getSequentialCommands(
				Arrays.asList(
						new SetHatchMech(HatchMechState.kClamped),
						new JankyGoToState(fieldPositions.cargoMiddleGoal, SuperStructure.iPosition.CARGO_PLACE))));

		dsCargo3.whenPressed(SequentialCommandFactory.getSequentialCommands(
				Arrays.asList(
						new SetHatchMech(HatchMechState.kClamped),
						new JankyGoToState(fieldPositions.cargoHighGoal, SuperStructure.iPosition.CARGO_PLACE_PITCHED_UP))));

		dsCargoShip.whenPressed(SequentialCommandFactory.getSequentialCommands(
				Arrays.asList(
						new SetHatchMech(HatchMechState.kClamped),
						new JankyGoToState(fieldPositions.cargoMiddleGoal.plus(LengthKt.getInch(2)), SuperStructure.iPosition.CARGO_DOWN))));

		// hatch presets
		primaryRightAnalogButton.whileHeld(new HybridDriverAssist(5));

		dsHatch1.whenPressed(SequentialCommandFactory.getSequentialCommands(
				Arrays.asList(
						new SetHatchMech(HatchMechState.kClamped),
						new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH))));

		dsHatch2.whenPressed(SequentialCommandFactory.getSequentialCommands(
				Arrays.asList(
						new SetHatchMech(HatchMechState.kClamped),
						new JankyGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH))));
		dsHatch3.whenPressed(SequentialCommandFactory.getSequentialCommands(
				Arrays.asList(
						new SetHatchMech(HatchMechState.kClamped),
						new JankyGoToState(fieldPositions.hatchHighGoal, iPosition.HATCH_PITCHED_UP))));

		dsHatchIn.whenPressed(SequentialCommandFactory.getSequentialCommands(
				Arrays.asList(
						new SetHatchMech(HatchMechState.kClamped),
						new JankyGoToState(iPosition.HATCH_GRAB_INSIDE))));

		primaryDpadUp.whenPressed(new KillAuto());
		// primaryDpadLeft.whenPressed(new CloseSideRocket('L'));
		// primaryDpadDown.whenPressed(new ArmMove(iPosition.HATCH));

		// primaryDpadUp.whenPressed(new ElevatorMove(new ElevatorState(LengthKt.getInch(35))));



		
			// 			primaryDpadUp.whenPressed(SequentialCommandFactory.getSequentialCommands(
			// Arrays.asList(
			// 		new SetHatchMech(HatchMechState.kOpen),
			// 		new SetGearCommand(Gear.HIGH),
			// 		new SetElevatorGear(ElevatorGear.LOW))));

		// dsJogUp.whenPressed(new JogElevator(LengthKt.getInch(0.5), true));
		// dsJogDown.whenPressed(new JogElevator(LengthKt.getInch(0.5), false));

		// dsJogUp.whenPressed(new JogElbow(RoundRotation2d.getDegree(2)));
		// dsJogDown.whenPressed(new JogElbow(RoundRotation2d.getDegree(-2)));

	}

	public boolean getWaiterButton() {
		return primaryBButton.get();
	}

	public Joystick getPrimary() {
		return primaryJoystick;
	}

	// public Joystick getSecondary() {
	// return secondaryJoystick;
	// }

	// public List<Joystick> getAllSticks() {
	// return Arrays.asList(getPrimary(), getSecondary());
	// }

	public enum OperatorControllers {
		PRIMARY, SECONDARY;
	}

	public void setRumble(RumbleType side, Joystick stick, double value) {
		value = Util.limit(value, 0, 1);
		stick.setRumble(side, value);
	}

	public void setAllRumble(double value) {
		setRumble(RumbleType.kLeftRumble, getPrimary(), value);
		// setRumble(RumbleType.kLeftRumble, getSecondary(), value);

		setRumble(RumbleType.kRightRumble, getPrimary(), value);
		// setRumble(RumbleType.kRightRumble, getSecondary(), value);
	}

	public double getForwardAxis() {
		return -1 * primaryJoystick.getRawAxis(RobotConfig.controls.forward_axis);
	}

	public double getTurnAxis() {
		return primaryJoystick.getRawAxis(RobotConfig.controls.turn_axis);
	}

	// public double getIntakeAxis() {
	// 	// return (secondaryJoystick.getRawButton(xboxmap.Buttons.RB_BUTTON)) ? 1 * 1 : 0;\
	// 	var inVal = driverStation.getRawButton(9) ? 1 : -1;
	// 	var outVal = driverStation.getRawButton(11) ? 1 : -1;
	// 	return inVal - outVal;
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

	// public double getWristAxis() {
	// return secondaryJoystick.getRawAxis(5);
	// }

	// public double getElbowAxis() {
	// return (secondaryJoystick.getRawAxis(2) - secondaryJoystick.getRawAxis(3)) * -1; // triggers
	// }

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
		return driverStation.getRawAxis(1);
	}

	public double getCargoSpeed() {
		return driverStation.getRawAxis(0) * 1;
	}

	// public double getElevatorAxis() {
	// return 0;
	// return secondaryJoystick.getRawAxis(RobotConfig.controls.xbox_elevator_axis) * -1;
	// }

	public double getElevatorDS() {
		var upPower = driverStation.getRawButton(9) ? 1 : -1;
		var downPower = driverStation.getRawButton(10) ? 1 : -1;
		return (upPower - downPower) * 0.06;
	}

	// public double getThrottleAxis() {
	// return 0;
	// }//secondaryJoystick.getRawAxis(RobotConfig.controls.throttle_elevator_axis); }

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
