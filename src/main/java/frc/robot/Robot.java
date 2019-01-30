package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auto.AutoMotion;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LIDARSubsystem;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.Superstructure;

/**
 * Main robot class. There shouldn't be a *ton* of stuff here, mostly init
 * functions and smartdashboard stuff.
 *
 * @author Matthew Morley
 */
public class Robot extends TimedRobot {
  public static SendableChooser<AutoMotion.mGoalHeight> mGh;
  public static OI m_oi;
  public static double startingDistance;
  public static double elevator_setpoint = 0;
  public static double wrist_setpoint = 0;

  public static boolean intakeOpen = false; // TODO I'm aware this shouldn't go here, I'll rewrite the intake subsystem
                                            // later

  public static boolean arcade_running = false;
  public static Intake intake = new Intake();
  // public static Elevator elevator = new Elevator();
  public static DriveTrain drivetrain = new DriveTrain();
  public static Superstructure superstructure = new Superstructure();  
  // public static Wrist wrist = new Wrist();
  public static AHRS gyro = new AHRS(SPI.Port.kMXP);
  public static LimeLight limelight = new LimeLight();
  public static LIDARSubsystem lidarSubsystem = new LIDARSubsystem();
  /** Poorly named Operator Input value for Robot */
  // public static TerribleLogger logger = new TerribleLogger();

  
  public static SendableChooser<AutoMotion> backupAutoSelect = new SendableChooser<AutoMotion>();
  private static DoubleSolenoid shifterDoubleSolenoid = new DoubleSolenoid(9, 7, 3);
  private static DoubleSolenoid intakeDoubleSolenoid = new DoubleSolenoid(9, 0, 6);

  // public static ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

  public static AutoMotion m_auto;

  SendableChooser<Command> m_chooser = new SendableChooser<Command>();

  Compressor compressor = new Compressor(9);

  

  // Various pneumatic shifting methods
  public static void drivetrain_shift_high() {
    shifterDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
  }

  public static void drivetrain_shift_low() {
    shifterDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public static void intake_close() {
    intakeDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
    intakeOpen = false;
  }

  public static void intake_open() {
    intakeDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    intakeOpen = true;
  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    m_oi = new OI();

    mGh = new SendableChooser<AutoMotion.mGoalHeight>();
    mGh.setDefaultOption("Low", AutoMotion.mGoalHeight.LOW);
    mGh.addOption("Middle", AutoMotion.mGoalHeight.MIDDLE);
    mGh.addOption("High", AutoMotion.mGoalHeight.HIGH);
    mGh.addOption("Dropped into the cargo ship", AutoMotion.mGoalHeight.OVER);
    SmartDashboard.putData("Goal Height", mGh);
    SmartDashboard.putData("Backup Selector (Will not be used in most cases)", backupAutoSelect);

    compressor.setClosedLoopControl(true);

    drivetrain.init();
    // elevator.init();
    // wrist.init();
    gyro.reset();

    startingDistance = drivetrain.getLeftDistance();

    switch (RobotConfig.auto.auto_gear) {
    case HIGH:
      drivetrain.setHighGear();
    case LOW:
      drivetrain.setLowGear();
    default:
      drivetrain.setHighGear();
    }

    SmartDashboard.putData(drivetrain);
    SmartDashboard.putData(superstructure); 
    SmartDashboard.putData(intake);
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {
    if (RobotConfig.auto.auto_gear == Gear.LOW) {
      drivetrain.setLowGear();
    } else if (RobotConfig.auto.auto_gear == Gear.HIGH) {
      drivetrain.setHighGear();
    } else {
      System.out.println("default auto gear " + RobotConfig.auto.auto_gear + " is not a valid choice!");
    }
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    // DriveTrajectoryPathfinder meme = new DriveTrajectoryPathfinder("file");
    // meme.start();

    gyro.reset(); // Reset the current gyro heading to zero
    drivetrain.zeroEncoders();

    if (RobotConfig.auto.auto_gear == Gear.LOW) {
      drivetrain.setLowGear();
    } else if (RobotConfig.auto.auto_gear == Gear.HIGH) {
      drivetrain.setHighGear();
    } else {
      System.out.println("default auto gear " + RobotConfig.auto.auto_gear + " is not a valid choice!");
    }
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
    if (m_auto != null) {
      m_auto.getBigCommandGroup().cancel();
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
  public void testPeriodic() { }

  /**
   * This function is called every robot packet, no matter the mode. Use // * this
   * for items like diagnostics that you want ran during disabled, // *
   * autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
    // TODO make a function or class that does all this calculation for us
    SmartDashboard.putNumber("get forward axis", m_oi.getForwardAxis());
    SmartDashboard.putNumber("get turn axis", m_oi.getTurnAxis());
    // SmartDashboard.putenum("Drivetrain gear", drivetrain.current_gear);
    // SmartDashboard.putNumber("setVelocityRight output: ",
    // encoderlib.distanceToRaw(12/12, 4096, 6/12) / 10 ); // This *should* return 1
    // ft/sec to raw/0.1 sec
    SmartDashboard.putNumber("target left speed raw", ((m_oi.getForwardAxis() * 4) / (Math.PI * 6 / 12)) * 4096 / 10);
    
    SmartDashboard.putNumber("Left talon speed", drivetrain.m_left_talon.getSelectedSensorVelocity(0));
    SmartDashboard.putNumber("Left talon error", drivetrain.m_left_talon.getClosedLoopError(0));
    SmartDashboard.putNumber("Right talon speed", drivetrain.m_right_talon.getSelectedSensorVelocity(0));
    SmartDashboard.putNumber("Right talon error", drivetrain.m_right_talon.getClosedLoopError(0));

    SmartDashboard.putNumber("Intake target speed per OI:", m_oi.getIntakeSpeed());

    // SmartDashboard.putNumber("Throttle output", throttle.getRawAxis(1));
    // SmartDashboard.putNumber("Elevator setpoint", 20000);
    // SmartDashboard.putNumber("Elevator height", elevator.getHeight());
    // SmartDashboard.putNumber("Elevator error", 4096 - elevator.getHeight());

    SmartDashboard.putBoolean("Arcade command running", arcade_running);

    // SmartDashboard.putNumber("Wrist angle setpoint", wrist_setpoint);
    // SmartDashboard.putNumber("Wrist talon pos",
    // elevator.elevator_talon.getSelectedSensorPosition(0));
    // SmartDashboard.putNumber("Wrist error",
    // elevator.elevator_talon.getClosedLoopError(0));
    // SmartDashboard.putNumber("Wrist angle (deg)", wrist.getAngle());
    // SmartDashboard.putNumber("Wrist angular velocity (deg/s)",
    // wrist.getAngularVelocity());

    SmartDashboard.putNumber("Current Gyro angle", gyro.getAngle());

    // SmartDashboard.putString("Limelight Ntables", LimeLight.getData().toString());

    // Limelight stuff
    double[] limelightdata = limelight.getData();

    SmartDashboard.putNumber("Vision targets?", limelightdata[0]);
    SmartDashboard.putNumber("Horizontal offset", limelightdata[1]);
    SmartDashboard.putNumber("Vertical offset", limelightdata[2]);
    SmartDashboard.putNumber("Target area", limelightdata[3]);
    SmartDashboard.putNumber("Target skew", limelightdata[4]);
    SmartDashboard.putNumber("Vision pipeline latency", limelightdata[5]);

    // logger.update();
  }

}
