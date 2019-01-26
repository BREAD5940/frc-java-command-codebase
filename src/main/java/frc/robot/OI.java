package frc.robot;

import java.io.File;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.groups.visionTest;
import frc.robot.commands.subsystems.drivetrain.DriveShiftHigh;
import frc.robot.commands.subsystems.drivetrain.DriveShiftLow;
import frc.robot.commands.subsystems.drivetrain.PurePursuit;
// import frc.robot.commands.subsystems.drivetrain.RamsetePathFollower;
// import frc.robot.commands.subsystems.drivetrain.RunDriveMotionPlanner;
import frc.robot.commands.subsystems.elevator.SetElevatorHeight;
import frc.robot.commands.subsystems.intake.CloseClamp;
import frc.robot.commands.subsystems.intake.OpenClamp;
import frc.robot.lib.motion.PathfinderTrajectory;
import frc.robot.subsystems.DriveTrain.Gear;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;


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

  private Button shift_up_button = new JoystickButton(primaryJoystick, RobotConfig.controls.shift_up_button);
  private Button shift_down_button = new JoystickButton(primaryJoystick, RobotConfig.controls.shift_down_button);
  private Button open_clamp_button = new JoystickButton(secondaryJoystick, xboxmap.Buttons.A_BUTTON);
  private Button close_clamp_button = new JoystickButton(secondaryJoystick, xboxmap.Buttons.Y_BUTTON);
  Button turnAutoButton = new JoystickButton(secondaryJoystick, xboxmap.Buttons.B_BUTTON);
  Button autobutton2 = new JoystickButton(secondaryJoystick, xboxmap.Buttons.X_BUTTON);
  Button autobutton3 = new JoystickButton(secondaryJoystick, xboxmap.Buttons.Y_BUTTON);


  // TODO change these to a button console once created
  Button auto_place_cargo_cargo_button = new JoystickButton(secondaryJoystick, xboxmap.Buttons.X_BUTTON);
  Button auto_place_hatch_cargo_button = new JoystickButton(secondaryJoystick, xboxmap.Buttons.B_BUTTON);
  Button auto_place_cargo_rocket_button = new JoystickButton(primaryJoystick, xboxmap.Buttons.Y_BUTTON);
  Button auto_place_hatch_rocket_button = new JoystickButton(primaryJoystick, xboxmap.Buttons.A_BUTTON);
  Button auto_grab_hatch_button = new JoystickButton(primaryJoystick, xboxmap.Buttons.LEFT_START_BUTTON);
  Button auto_grab_cargo_button = new JoystickButton(primaryJoystick, xboxmap.Buttons.RIGHT_START_BUTTON);

  // File file = new File("/home/lvuser/deploy/paths/test.pf1.csv");
  // Trajectory trajectory = Pathfinder.readFromCSV(file);
  // PathfinderTrajectory pftraj = /*new PathfinderTrajectory(trajectory);*/ PathfinderTrajectory.readFromTrajectory(trajectory);

  public OI() {
    shift_up_button.whenPressed(new DriveShiftHigh());
    shift_down_button.whenPressed(new DriveShiftLow());
    // open_clamp_button.whenPressed(new OpenClamp());
    // close_clamp_button.whenPressed(new CloseClamp());
    auto_place_cargo_cargo_button.whenPressed(new SetElevatorHeight(9.316859344380049 - 1.5 ) );
    auto_place_hatch_cargo_button.whenPressed(new SetElevatorHeight(30));

    auto_place_cargo_rocket_button.whenPressed(new visionTest());
    // auto_place_hatch_rocket_button.whenPressed(new RunDriveMotionPlanner(  pftraj ) );


    // auto_place_hatch_cargo_button.whenPressed(new RunAuto(mGoalType.CARGO_HATCH, AutoMotion.mGoalHeight.LOW));
    // auto_place_cargo_rocket_button.whenPressed(new RunAuto(mGoalType.ROCKET_CARGO, AutoMotion.mGoalHeight.LOW));
    // auto_place_hatch_rocket_button.whenPressed(new RunAuto(mGoalType.ROCKET_HATCH, AutoMotion.mGoalHeight.LOW));
    // auto_grab_hatch_button.whenPressed(new RunAuto(mGoalType.RETRIEVE_HATCH, AutoMotion.mGoalHeight.LOW));
    // auto_grab_cargo_button.whenPressed(new RunAuto(mGoalType.RETRIEVE_CARGO, AutoMotion.mGoalHeight.LOW));
    // turnAutoButton.whenPressed(new PurePursuitPathCommand());
    // autobutton2.whenPressed(new RamsetePathFollower("filePath"));
    // autobutton3.whenPressed(new DriveTrajectoryPathfinder("mFile"));
    open_clamp_button.whenPressed(new OpenClamp());
    close_clamp_button.whenPressed(new CloseClamp());
    // open_clamp_button.whenPressed(new PurePursuit());
  }

  public double getForwardAxis() { return -1 * primaryJoystick.getRawAxis(RobotConfig.controls.forward_axis); }
  public double getTurnAxis() { return primaryJoystick.getRawAxis(RobotConfig.controls.turn_axis); }
  public double getIntakeAxis() { return secondaryJoystick.getRawAxis(RobotConfig.controls.intakeAxis); }
  public double getOuttakeAxis() { return secondaryJoystick.getRawAxis(RobotConfig.controls.outtakeAxis); }
  /**
   * Get intake speed is the difference between intake and outtake axis speeds
   */
  public double getIntakeSpeed(){ return getIntakeAxis() - getOuttakeAxis(); }
  public double getElevatorAxis(){ return secondaryJoystick.getRawAxis(RobotConfig.controls.xbox_elevator_axis) * -1; }
  public double getThrottleAxis() { return secondaryJoystick.getRawAxis(RobotConfig.controls.throttle_elevator_axis); }

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
