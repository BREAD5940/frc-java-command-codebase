/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoSelector;
import frc.robot.auto.actions.auto_DRIVE;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.auto.groups.auto_SQUARE;
// import frc.robot.subsystems.intake;
import frc.robot.subsystems.Wrist;
// import frc.robot.commands.drivetrain_shift_high;
// import frc.robot.commands.drivetrain_shift_low;
import frc.robot.auto.AutoPath;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
// import frc.robot.commands.arcade_drive;
// import frc.robot.subsystems.*;
// import frc.robot.commands.*;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // WARNING see this link:
  // https://github.com/mr-glt/FRC-2017-Command/blob/master/src/main/java/org/usfirst/frc/team6027/robot/subsystems/DriveTrain.java
  // In the init, they have this instead:
  // 	public static DriveTrain drivetrain;
  // Rather than making a new drivetrain();
  // TODO does this have anything to do with the weird command errors? Or what about making it final?

  public static boolean arcade_running = false;
  public static DriveTrain drivetrain = new DriveTrain();
  public static Intake intake = new Intake();
  public static Elevator elevator = new Elevator();
  public static Wrist wrist = new Wrist();
  public static RobotConfig robotconfig = new RobotConfig();
  public static OI m_oi;

  public static double elevator_setpoint = 0;
  public static double wrist_setpoint = 0;
  private static DoubleSolenoid shifterDoubleSolenoid = new DoubleSolenoid(9, 7, 3);
  private static DoubleSolenoid intakeDoubleSolenoid = new DoubleSolenoid(9, 0, 6);

  public static ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
  
  double startingDistance;

  AutoSelector autoSelect;

  public static double defaultAutoSpeed = RobotConfig.drive_auto_forward_velocity_max;
  public static AutoPath m_auto;

  public static void drivetrain_shift_high(){ shifterDoubleSolenoid.set(DoubleSolenoid.Value.kForward); }
  public static void drivetrain_shift_low(){ shifterDoubleSolenoid.set(DoubleSolenoid.Value.kReverse); }
  public static void intake_close(){ intakeDoubleSolenoid.set(DoubleSolenoid.Value.kForward); }
  public static void intake_open(){ intakeDoubleSolenoid.set(DoubleSolenoid.Value.kReverse); }

  public static double rawSpeedRight;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();
    Compressor compressor = new Compressor(9);
    compressor.setClosedLoopControl(true);
    
    drivetrain.init();
    elevator.init();
    wrist.init();
    gyro.reset();

    System.out.println("Hi!");

    startingDistance = drivetrain.getLeftDistance();

    // m_chooser.addDefault("Default Auto", new auto_action_DRIVE(3, "high", 5, 30));
    // m_chooser.addObject("Square Auto", new auto_action_SQUARE(2));
    // chooser.addObject("My Auto", new MyAutoCommand());



  }

  /**
   * This function is called every robot packet, no matter the mode. Use
  //  * this for items like diagnostics that you want ran during disabled,
  //  * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("get forward axis", m_oi.getForwardAxis());
    SmartDashboard.putNumber("get turn axis", m_oi.getTurnAxis());
    SmartDashboard.putString("Drivetrain gear", drivetrain.current_gear); 
    // SmartDashboard.putNumber("setVelocityRight output: ", encoderlib.distanceToRaw(12/12, 4096, 6/12) / 10 ); // This *should* return 1 ft/sec to raw/0.1 sec
    SmartDashboard.putNumber("target left speed raw",  
      ((m_oi.getForwardAxis() * 4) / (Math.PI * 6 / 12)) * 4096 / 10
    );

    SmartDashboard.putNumber("Left talon speed", drivetrain.m_left_talon.getSelectedSensorVelocity(0));
    SmartDashboard.putNumber("Left talon error", drivetrain.m_left_talon.getClosedLoopError(0));
    SmartDashboard.putNumber("Right talon speed", drivetrain.m_right_talon.getSelectedSensorVelocity(0));
    SmartDashboard.putNumber("Right talon error", drivetrain.m_right_talon.getClosedLoopError(0));

    SmartDashboard.putNumber("Intake target speed per OI:", m_oi.getIntakeSpeed());

    // SmartDashboard.putNumber("Throttle output", throttle.getRawAxis(1));
    SmartDashboard.putNumber("Elevator setpoint", 20000);
    SmartDashboard.putNumber("Elevator height", elevator.getHeight());
    SmartDashboard.putNumber("Elevator error", 4096-elevator.getHeight());

    SmartDashboard.putBoolean("Arcade command running", arcade_running);

    // SmartDashboard.putNumber("Wrist angle setpoint", wrist_setpoint); 
    // SmartDashboard.putNumber("Wrist talon pos", elevator.elevator_talon.getSelectedSensorPosition(0));
    // SmartDashboard.putNumber("Wrist error", elevator.elevator_talon.getClosedLoopError(0));
    // SmartDashboard.putNumber("Wrist angle (deg)", wrist.getAngle());
    // SmartDashboard.putNumber("Wrist angular velocity (deg/s)", wrist.getAngularVelocity());

    SmartDashboard.putNumber("Current Gyro angle", gyro.getAngle());
    
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString code to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons
   * to the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    autoSelect = new AutoSelector();
    SmartDashboard.putData("Robot Location Chooser", autoSelect.rbLoc);
    SmartDashboard.putData("Goal Chooser", autoSelect.usrGoal);
    SmartDashboard.putData("Number of Cubes", autoSelect.usrCubes);
    SmartDashboard.putData("Backup Selector (Will not be used in most cases)", autoSelect.backupAutoSelect);
    m_auto = autoSelect.choosePath();

    m_auto.getCommandGroup().start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    
    
  }

  @Override
  public void teleopInit() {
    // drivetrain.init();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_auto != null) {
        m_auto.getCommandGroup().cancel();
    }

    // new drivetrain_shift_high();
    // new drivetrain_shift_low();
    // shifter.set(DoubleSolenoid.Value.kReverse);


    // final arcade_drive arcade = new arcade_drive();

    // new arcade_drive();


  }

  /**
   * This function is called periodically during operator control.
   * 
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    
    // double target_intake_speed = m_oi.getIntakeSpeed() / 1;
    // // intake.setSpeed(target_intake_speed);
    // intake.talon_left.set(ControlMode.PercentOutput, target_intake_speed);
    // intake.talon_right.set(ControlMode.PercentOutput, -target_intake_speed);


  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
