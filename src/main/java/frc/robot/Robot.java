package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.actions.auto_DriveDistance;
import frc.robot.auto.actions.auto_DriveTrajectoryPathfinder;
import frc.robot.lib.EncoderLib;
// import frc.robot.lib.TerribleLogger;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
// import frc.robot.subsystems.Wrist;

import frc.robot.subsystems.DriveTrain.Gear;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import frc.robot.subsystems.DriveTrain.Gear;

/**
 * Main robot class. There shouldn't be a *ton* of stuff here, mostly
 * init functions and smartdashboard stuff. 
 * 
 * @author Matthew Morley
 */
public class Robot extends TimedRobot {

  public static boolean arcade_running = false;
  public static DriveTrain drivetrain = new DriveTrain();
  public static Intake intake = new Intake();
  public static Elevator elevator = new Elevator();
  // public static Wrist wrist = new Wrist();
  public static LimeLight limelight = new LimeLight();
  public static OI m_oi;
  // public static TerribleLogger logger = new TerribleLogger();

  public static double elevator_setpoint = 0;
  public static double wrist_setpoint = 0;
  private static DoubleSolenoid shifterDoubleSolenoid = new DoubleSolenoid(9, 7, 3);
  private static DoubleSolenoid intakeDoubleSolenoid = new DoubleSolenoid(9, 0, 6);

  public static ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  Compressor compressor = new Compressor(9);
  
  Command m_autonomousCommand;
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public static double startingDistance;

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
    // wrist.init();
    gyro.reset();

    startingDistance = drivetrain.getLeftDistance();


    Waypoint[] points = new Waypoint[] {
      new Waypoint(0, 0, Pathfinder.d2r(90)),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
      new Waypoint(10, 10, Pathfinder.d2r(0))                        // Waypoint @ x=-2, y=-2, exit angle=45 degrees
      // new Waypoint(0, 0, 0)                           // Waypoint @ x=0, y=0,   exit angle=0 radians
    };
    
    Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.02, 1.7, 2.0, 60.0);
    Trajectory trajectory = Pathfinder.generate(points, config);

    TankModifier modifier = new TankModifier(trajectory).modify(2);

    m_chooser.setDefaultOption("Drive Auto", new auto_DriveDistance(10));
    m_chooser.addOption("This is a test", new auto_DriveDistance(2));
    m_chooser.addOption("Pathfinder test", new auto_DriveTrajectoryPathfinder("trajectory"));
    SmartDashboard.putData("auto mode", m_chooser);
    
    if ( RobotConfig.auto.auto_gear == Gear.HIGH ) { drivetrain.setHighGear(); }
    else { drivetrain.setLowGear(); }
    // else { System.out.println("default auto gear " + RobotConfig.auto.auto_gear + " is not a valid choice!"); }

    System.out.println("Robot has been initilized!");

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    if ( RobotConfig.auto.auto_gear == Gear.LOW ) { drivetrain.setLowGear(); }
    else if ( RobotConfig.auto.auto_gear == Gear.HIGH ) { drivetrain.setHighGear(); }
    else { System.out.println("default auto gear " + RobotConfig.auto.auto_gear + " is not a valid choice!"); }
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
    
    if ( RobotConfig.auto.auto_gear == Gear.LOW) {
      drivetrain.setLowGear();
    }
    else if ( RobotConfig.auto.auto_gear == Gear.HIGH ) { 
      drivetrain.setHighGear(); 
    }
    else { System.out.println("default auto gear " + RobotConfig.auto.auto_gear + " is not a valid choice!"); }

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
    // SmartDashboard.putenum("Drivetrain gear", drivetrain.current_gear); 
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

    // Limelight stuff
    // double[] limelightdata = limelight.getData();

    // SmartDashboard.putNumber("Vision targets?", limelightdata[0]);
    // SmartDashboard.putNumber("Horizontal offset", limelightdata[1]);
    // SmartDashboard.putNumber("Vertical offset", limelightdata[2]);
    // SmartDashboard.putNumber("Target area", limelightdata[3]);
    // SmartDashboard.putNumber("Target skew", limelightdata[4]);
    // SmartDashboard.putNumber("Vision pipeline latency", limelightdata[5]);

    // logger.update();
  }

}

