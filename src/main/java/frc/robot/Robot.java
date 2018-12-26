/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.actions.auto_DriveDistance;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static boolean arcade_running = false;
  public static DriveTrain drivetrain = new DriveTrain();
  public static Intake intake = new Intake();
  public static Elevator elevator = new Elevator();
  public static Wrist wrist = new Wrist();
  public static OI m_oi;

  public static double elevator_setpoint = 0;
  public static double wrist_setpoint = 0;
  private static DoubleSolenoid shifterDoubleSolenoid = new DoubleSolenoid(9, 7, 3);
  private static DoubleSolenoid intakeDoubleSolenoid = new DoubleSolenoid(9, 0, 6);

  public static ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  Compressor compressor = new Compressor(9);
  
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public static double startingDistance;
  public static double defaultAutoSpeed = RobotConfig.drive_auto_forward_velocity_max;

  // Various pneumatic shifting methods
  public static void drivetrain_shift_high(){ shifterDoubleSolenoid.set(DoubleSolenoid.Value.kForward); }
  public static void drivetrain_shift_low(){ shifterDoubleSolenoid.set(DoubleSolenoid.Value.kReverse); }
  public static void intake_close(){ intakeDoubleSolenoid.set(DoubleSolenoid.Value.kForward); }
  public static void intake_open(){ intakeDoubleSolenoid.set(DoubleSolenoid.Value.kReverse); }

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();

    compressor.setClosedLoopControl(true);
    
    drivetrain.init();
    elevator.init();
    wrist.init();
    gyro.reset();

    startingDistance = drivetrain.getLeftDistance();

    m_chooser.addDefault("Drive Auto", new auto_DriveDistance(10));
    m_chooser.addObject("This is a test", new auto_DriveDistance(2));

    if ( RobotConfig.default_auto_gear == "low" ) { drivetrain.setLowGear(); }
    else if ( RobotConfig.default_auto_gear == "high" ) { drivetrain.setHighGear(); }
    else { System.out.println("default auto gear " + RobotConfig.default_auto_gear + " is not a valid choice!"); }

    System.out.println("Robot has been initilized!");

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    if ( RobotConfig.default_auto_gear == "low" ) { drivetrain.setLowGear(); }
    else if ( RobotConfig.default_auto_gear == "high" ) { drivetrain.setHighGear(); }
    else { System.out.println("default auto gear " + RobotConfig.default_auto_gear + " is not a valid choice!"); }
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
    m_autonomousCommand = m_chooser.getSelected(); // set the command to what the sendable chooser gets

    gyro.reset(); // Reset the current gyro heading to zero
    drivetrain.zeroEncoders();
    
    if ( RobotConfig.default_auto_gear == "low" ) { drivetrain.setLowGear(); }
    else if ( RobotConfig.default_auto_gear == "high" ) { drivetrain.setHighGear(); }
    else { System.out.println("default auto gear " + RobotConfig.default_auto_gear + " is not a valid choice!"); }

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.start();
    }
    // new auto_action_DRIVE(3, "high", 5, 30);

    // TODO so this doesnt work  for some reason, TODO figure this out
    // System.out.println("Trying to call the auto action drive...");
    // new auto_action_DRIVE(5, "high", 5, 30);

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
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  // TODO reset subsystems on teleop init?
  }

  /**
   * This function is called periodically during operator control.
   * 
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
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

}

