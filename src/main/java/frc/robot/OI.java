package frc.robot;

import java.util.Arrays;

import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.auto.routines.TeleopCommands;
import frc.robot.commands.subsystems.drivetrain.HybridDriverAssist;
import frc.robot.commands.subsystems.drivetrain.SetGearCommand;
import frc.robot.commands.subsystems.superstructure.JankyGoToState;
import frc.robot.commands.subsystems.superstructure.PassThrough;
import frc.robot.commands.subsystems.superstructure.SetHatchMech;
import frc.robot.lib.AnalogButton;
import frc.robot.lib.DPadButton;
import frc.robot.lib.ParallelRaceGroup;
import frc.robot.lib.motion.Util;
import frc.robot.lib.obj.factories.SequentialCommandFactory;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.subsystems.Intake.HatchMechState;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

/**
 * Operator Input not Out-In This class is the glue that binds the controls on
 * the physical operator interface to the commands and command groups that allow
 * control of the robot. For use with commands and stuff.
 *
 * @author Matthew Morley
 */
public class OI {

	private Joystick primaryJoystick = new Joystick(RobotConfig.controls.primary_joystick_port);
	private Joystick driverStation = new Joystick(5); // TODO make a constant

	private Button shift_up_button = new JoystickButton(primaryJoystick, RobotConfig.controls.shift_up_button);
	private Button shift_down_button = new JoystickButton(primaryJoystick, RobotConfig.controls.shift_down_button);

	// TODO change these to a button console once created

	Button primaryYButton = new JoystickButton(primaryJoystick, xboxmap.Buttons.Y_BUTTON);
	Button primaryAButton = new JoystickButton(primaryJoystick, xboxmap.Buttons.A_BUTTON);
	Button primaryXButton = new JoystickButton(primaryJoystick, xboxmap.Buttons.X_BUTTON);
	Button primaryBButton = new JoystickButton(primaryJoystick, xboxmap.Buttons.B_BUTTON);

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

	Button dsPassThroughFrontToBack = new JoystickButton(driverStation, 4);

	public OI() {

		shift_up_button.whenPressed(new SetGearCommand(Gear.HIGH));
		shift_down_button.whenPressed(new SetGearCommand(Gear.LOW));

		// cargo presets
		dsCargoIn.whenPressed(
				SequentialCommandFactory.getSequentialCommands(Arrays.asList(new SetHatchMech(HatchMechState.kOpen),
						new JankyGoToState(LengthKt.getInch(13.25), SuperStructure.iPosition.CARGO_GRAB))));

		dsCargo1.whenPressed(
				SequentialCommandFactory.getSequentialCommands(Arrays.asList(new SetHatchMech(HatchMechState.kClamped),
						new JankyGoToState(fieldPositions.cargoLowGoal, SuperStructure.iPosition.CARGO_PLACE))));

		dsCargo2.whenPressed(
				SequentialCommandFactory.getSequentialCommands(Arrays.asList(new SetHatchMech(HatchMechState.kClamped),
						new JankyGoToState(fieldPositions.cargoMiddleGoal, SuperStructure.iPosition.CARGO_PLACE))));

		dsCargo3.whenPressed(
				SequentialCommandFactory.getSequentialCommands(Arrays.asList(new SetHatchMech(HatchMechState.kClamped),
						new JankyGoToState(fieldPositions.cargoHighGoal, SuperStructure.iPosition.CARGO_PLACE_PITCHED_UP))));

		dsCargoShip.whenPressed(SequentialCommandFactory
				.getSequentialCommands(Arrays.asList(new SetHatchMech(HatchMechState.kClamped), new JankyGoToState(
						fieldPositions.cargoMiddleGoal.plus(LengthKt.getInch(2)), SuperStructure.iPosition.CARGO_DOWN))));

		// hatch presets
//		primaryBButton.whileHeld(new HybridDriverAssist());
		 primaryRightAnalogButton.whileHeld(new HybridDriverAssist());

		dsHatch1.whenPressed(SequentialCommandFactory.getSequentialCommands(Arrays.asList(
				new SetHatchMech(HatchMechState.kClamped), new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH))));

		dsHatch2.whenPressed(
				SequentialCommandFactory.getSequentialCommands(Arrays.asList(new SetHatchMech(HatchMechState.kClamped),
						new JankyGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH))));
		dsHatch3.whenPressed(
				SequentialCommandFactory.getSequentialCommands(Arrays.asList(new SetHatchMech(HatchMechState.kClamped),
						new JankyGoToState(fieldPositions.hatchHighGoal, iPosition.HATCH_PITCHED_UP))));

		dsHatchIn.whenPressed(SequentialCommandFactory.getSequentialCommands(
				Arrays.asList(new SetHatchMech(HatchMechState.kClamped), new JankyGoToState(iPosition.HATCH_GRAB_INSIDE))));

		var testMeme = new CommandGroup();
		testMeme.addSequential(new ParallelRaceGroup(() -> (Robot.m_oi.getPrimary().getRawButton(xboxmap.Buttons.A_BUTTON)),
				new TeleopCommands()));
		testMeme.addSequential(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));

		primaryDpadDown.whenPressed(testMeme);

		//		primaryXButton.whenPressed(new PassThrough(SuperStructure.getInstance(), () -> SuperStructure.getInstance().getCurrentState().getElbowAngle().getDegree() >= -90));

		//		dsTogglePassThru.whenPressed(new PrintCommand("nelasdfljkasdlk;"));
		primaryDpadUp.whenPressed(new PassThrough.BackToFront(SuperStructure.getInstance()));
		primaryDpadDown.whenPressed(new PassThrough.FrontToBack(SuperStructure.getInstance()));

		dsPassThroughFrontToBack.whenPressed(new PassThrough.FrontToBack(SuperStructure.getInstance()));

	}

	public boolean getWaiterButton() {
		return primaryBButton.get();
	}

	public Joystick getPrimary() {
		return primaryJoystick;
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
		// setRumble(RumbleType.kLeftRumble, getSecondary(), value);

		setRumble(RumbleType.kRightRumble, getPrimary(), value);
		// setRumble(RumbleType.kRightRumble, getSecondary(), value);
	}

	public double getForwardAxis() {
				 return -1 * primaryJoystick.getRawAxis(RobotConfig.controls.forward_axis);
//		return getPrimary().getRawAxis(3) - getPrimary().getRawAxis(2);
	}

	public double getTurnAxis() {
				return primaryJoystick.getRawAxis(xboxmap.Axis.RIGHT_JOYSTICK_X);
//		return primaryJoystick.getRawAxis(0) + primaryJoystick.getRawAxis(xboxmap.Axis.RIGHT_JOYSTICK_X);
	}

	public double getDSElbowAxis() {
		return 0;// (driverStation.getRawAxis(DriverstationMap.Axes.elbowStick));
	}

	public double getDSElevatorAxis() {
		return 0;// (driverStation.getRawAxis(DriverstationMap.Axes.elevatorStick));
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

	public double getElevatorDS() {
		var upPower = driverStation.getRawButton(9) ? 1 : -1;
		var downPower = (driverStation.getRawButton(11) ? 1 : -1) * -1;

		var toReturn = (upPower + downPower) * 0.15;

		System.out.println(toReturn);
		return toReturn;
	}

}
