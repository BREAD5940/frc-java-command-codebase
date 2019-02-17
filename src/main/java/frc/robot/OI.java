package frc.robot;

import org.ghrobotics.lib.mathematics.units.Rotation2dKt;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.auto.MultiPathTest;
import frc.robot.commands.auto.RunAuto;
import frc.robot.commands.auto.Trajectories;
import frc.robot.commands.subsystems.drivetrain.SetGearCommand;
import frc.robot.commands.subsystems.drivetrain.TurnInPlace;
import frc.robot.lib.ChordedInput;
import frc.robot.lib.ChordedTrajectory;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;

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
	protected static Joystick dhd = new Joystick(RobotConfig.controls.dhd_port);

	private Button shift_up_button = new JoystickButton(primaryJoystick, RobotConfig.controls.shift_up_button);
	private Button shift_down_button = new JoystickButton(primaryJoystick, RobotConfig.controls.shift_down_button);

	Button yeetInACircleButton = new JoystickButton(primaryJoystick, xboxmap.Buttons.X_BUTTON);
	Button testAutoButton = new JoystickButton(primaryJoystick, xboxmap.Buttons.B_BUTTON);
	Button auto_grab_hatch_button = new JoystickButton(primaryJoystick, xboxmap.Buttons.LEFT_START_BUTTON);
	Button auto_grab_cargo_button = new JoystickButton(primaryJoystick, xboxmap.Buttons.RIGHT_START_BUTTON);
	Button test2button = new JoystickButton(primaryJoystick, xboxmap.Buttons.Y_BUTTON);

	Button runDHDInput = new JoystickButton(dhd, dhdMap.RUN);

	public OI() {
		shift_up_button.whenPressed(new SetGearCommand(Gear.HIGH));
		shift_down_button.whenPressed(new SetGearCommand(Gear.LOW));
		// open_clamp_button.whenPressed(new OpenClamp());
		// close_clamp_button.whenPressed(new CloseClamp());
		if (Trajectories.forward20Feet == null)
			Trajectories.generateAllTrajectories();
		// testAutoButton.whenPressed(DriveTrain.getInstance().followTrajectory(Trajectories.generatedTrajectories.get("test"), true));

		test2button.whenPressed(DriveTrain.getInstance().followTrajectory(Trajectories.forward20Feet, TrajectoryTrackerMode.RAMSETE, true));

		testAutoButton.whenPressed(new MultiPathTest());
		// yeetInACircleButton.whenPressed(DriveTrain.getInstance().followTrajectory(Trajectories.forward20Feet, true));
		yeetInACircleButton.whenPressed(new TurnInPlace(Rotation2dKt.getDegree(180), false));

		runDHDInput.whenPressed(new RunAuto(ChordedTrajectory.getTraject(Gear.HIGH), ChordedInput.getGT(), ChordedInput.getGH()));

	}

	public double getForwardAxis() {
		return -1 * primaryJoystick.getRawAxis(RobotConfig.controls.forward_axis);
	}

	public double getTurnAxis() {
		return primaryJoystick.getRawAxis(RobotConfig.controls.turn_axis);
	}

	public double getIntakeAxis() {
		return primaryJoystick.getRawAxis(RobotConfig.controls.intakeAxis);
	}

	public double getOuttakeAxis() {
		return primaryJoystick.getRawAxis(RobotConfig.controls.outtakeAxis);
	}

	public double getWristAxis() {
		return secondaryJoystick.getRawAxis(5);
	}

	public double getElbowAxis() {
		return secondaryJoystick.getRawAxis(3) - secondaryJoystick.getRawAxis(2); // triggers
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
	}
}
