package frc.robot;

import java.util.Arrays;
import java.util.List;

import org.ghrobotics.lib.debug.LiveDashboard;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auto.AutoMotion;
import frc.robot.commands.auto.Trajectories;
import frc.robot.commands.subsystems.drivetrain.ZeroSuperStructure;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.superstructure.SuperStructure;

/**
 * Main robot class. There shouldn't be a *ton* of stuff here, mostly init
 * functions and smartdashboard stuff.
 *
 * @author Matthew Morley
 */
public class Robot extends TimedRobot {
	public static SendableChooser<AutoMotion.GoalHeight> mGh;
	public static OI m_oi;
	// public static Intake intake = new Intake();
	// public static Elevator elevator = new Elevator();
	public static DriveTrain drivetrain = DriveTrain.getInstance();
	public static SuperStructure superstructure = SuperStructure.getInstance();
	// public static VisionProcessor visionProcessor = new VisionProcessor();
	public static LimeLight limelight = new LimeLight();
	// public static LIDARSubsystem lidarSubsystem = new LIDARSubsystem();
	public static DoubleSolenoid shifterDoubleSolenoid;
	public static DoubleSolenoid intakeDoubleSolenoid;
	public static DoubleSolenoid elevatorShifterDoubleSolenoid;
	public static AutoMotion m_auto;
	SendableChooser<Command> m_chooser = new SendableChooser<Command>();
	public static Compressor compressor = new Compressor(9);
	// public static Odometer odometry_;
	// public static DifferentialUltrasonicSensor differentialUltrasonicSensor = DifferentialUltrasonicSensor.getInstance();
	// private Logger logger;

	public static boolean intakeOpen = false; // TODO I'm aware this shouldn't go here, I'll rewrite the intake subsystem
												// later

	// Various pneumatic shifting methods
	public static void drivetrain_shift_high() {
		getShifterSolenoid().set(DoubleSolenoid.Value.kForward);
	}

	public static DoubleSolenoid getShifterSolenoid() {
		if (shifterDoubleSolenoid == null)
			shifterDoubleSolenoid = new DoubleSolenoid(9, 0, 1);
		return shifterDoubleSolenoid;
	}

	public static DoubleSolenoid getIntakeSolenoidInstance() {
		if (intakeDoubleSolenoid == null)
			intakeDoubleSolenoid = new DoubleSolenoid(9, 2, 3);
		return intakeDoubleSolenoid;
	}

	public static DoubleSolenoid getElevatorShifter() {
		if (elevatorShifterDoubleSolenoid == null) {
			elevatorShifterDoubleSolenoid = new DoubleSolenoid(9, 4, 5);
		}
		return elevatorShifterDoubleSolenoid;
	}

	public static Gear getDrivetrainGear() {
		return (getShifterSolenoid().get() == Value.kForward) ? Gear.HIGH : Gear.LOW;
	}

	public static Value getIntakeSolenoidState() {
		return intakeDoubleSolenoid.get();
	}

	public static void drivetrain_shift_low() {
		getShifterSolenoid().set(DoubleSolenoid.Value.kReverse);
	}

	public static void intake_close() {
		intakeDoubleSolenoid.set(DoubleSolenoid.Value.kForward);
	}

	public static void intake_open() {
		intakeDoubleSolenoid.set(DoubleSolenoid.Value.kReverse);
	}

	public static void setElevatorShifter(boolean isKForward) {
		DoubleSolenoid.Value value = (isKForward) ? Value.kForward : Value.kReverse;
		if (elevatorShifterDoubleSolenoid == null)
			elevatorShifterDoubleSolenoid = new DoubleSolenoid(9, 4, 5);
		if (value == null)
			value = DoubleSolenoid.Value.kForward; // TODO hack
		elevatorShifterDoubleSolenoid.set(value); // FIXME it's a hack
	}

	/**
	 * FIXME it's a hack, set the period to 25ms
	 */
	public Robot() {
		super(0.025d);
	}

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		if (drivetrain == null)
			drivetrain = DriveTrain.getInstance();
		// FIXME Jocelyn this might mess with auto stuff, will it? (I think no?)
		drivetrain.getLocalization().reset(new Pose2d(LengthKt.getFeet(5.5), LengthKt.getFeet(17), new Rotation2d(0f, 0f, false)));

		Trajectories.generateAllTrajectories();

		// logger = Logger.getInstance();
		m_oi = new OI();
		mGh = new SendableChooser<AutoMotion.GoalHeight>();
		mGh.setDefaultOption("Low", AutoMotion.GoalHeight.LOW);
		mGh.addOption("Middle", AutoMotion.GoalHeight.MIDDLE);
		mGh.addOption("High", AutoMotion.GoalHeight.HIGH);
		mGh.addOption("Dropped into the cargo ship", AutoMotion.GoalHeight.OVER);
		SmartDashboard.putData("Goal Height", mGh);
		SmartDashboard.putData(drivetrain);
		SmartDashboard.putData(superstructure);
		// SmartDashboard.putData(SuperStructure.intake);
		// SmartDashboard.putData(shifterDoubleSolenoid);

		compressor.setClosedLoopControl(true);

		drivetrain.init();
		// elevator.init();
		// wrist.init();
		drivetrain.zeroGyro();

		switch (RobotConfig.auto.auto_gear) {
		case HIGH:
			drivetrain.setHighGear();
			break;
		case LOW:
			drivetrain.setLowGear();
			break;
		default:
			drivetrain.setHighGear();
		}

		// odometry_ = Odometer.getInstance();
		// Thanks to RoboLancers for odometry code
		// odometry_ = Odometry.getInstance();
		// new Notifier(() -> {
		//     // odometry_.setCurrentEncoderPosition((DriveTrain.getInstance().getLeft().getEncoderCount() + DriveTrain.getInstance().getRight().getEncoderCount()) / 2.0);

		//     // Incrament the current encoder position by the average
		//     odometry_.setCurrentEncoderPosition((drivetrain.m_left_talon.getSelectedSensorPosition() + drivetrain.m_right_talon.getSelectedSensorPosition()) / 2.0);

		//     // odometry_.setDeltaPosition(RobotUtil.encoderTicksToFeets(odometry_.getCurrentEncoderPosition() - odometry_.getLastPosition()));
		//     odometry_.setDeltaPosition(EncoderLib.rawToDistance(odometry_.getCurrentEncoderPosition() - odometry_.getLastPosition(), RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION,
		//               (RobotConfig.driveTrain.left_wheel_effective_diameter / 12 + RobotConfig.driveTrain.right_wheel_effective_diameter / 12)/2.0));

		//     odometry_.setTheta(Math.toRadians(Pathfinder.boundHalfDegrees(gyro.getAngle())));

		//     odometry_.addX(Math.cos(odometry_.getTheta()) * odometry_.getDeltaPosition());
		//     odometry_.addY(Math.sin(odometry_.getTheta()) * odometry_.getDeltaPosition());

		//     odometry_.setLastPosition(odometry_.getCurrentEncoderPosition());
		// }).startPeriodic(0.02);
		SmartDashboard.putData("Zero elevator height:", new ZeroSuperStructure("elevator"));
		SmartDashboard.putData("Zero elbow angle:", new ZeroSuperStructure("elbow"));
		SmartDashboard.putData("Zero wrist angle:", new ZeroSuperStructure("wrist"));
		SmartDashboard.putData("Max elevator height:", new ZeroSuperStructure("maxElevator"));
		SmartDashboard.putData("Max wrist angle:", new ZeroSuperStructure("maxWrist"));
		SmartDashboard.putData("Min wrist angle:", new ZeroSuperStructure("minWrist"));
		SmartDashboard.putData("Max elbow angle:", new ZeroSuperStructure("maxElbow"));
		SmartDashboard.putData("Min elbow angle:", new ZeroSuperStructure("minElbow"));
		drivetrain.zeroEncoders();
		System.out.println("Robot init'ed and encoders zeroed!");

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

		drivetrain.setNeutralMode(NeutralMode.Coast);
		SuperStructure.lastSH = LengthKt.getInch(0);

	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		// DriveTrajectoryPathfinder meme = new DriveTrajectoryPathfinder("file");
		// meme.start();

		// drivetrain.gyro.reset(); // Reset the current gyro heading to zero
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

		// odometry_.setX(0);
		// odometry_.setY(0);
		drivetrain.zeroGyro();
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
	public void testPeriodic() {}

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
		drivetrain.getLocalization().update(); // TODO put me on a notifier?
		// long now = System.currentTimeMillis();

		SmartDashboard.putNumber("Robot X (feet) ", drivetrain.getLocalization().getRobotPosition().getTranslation().getX().getFeet());
		SmartDashboard.putNumber("Robot Y (feet) ", drivetrain.getLocalization().getRobotPosition().getTranslation().getY().getFeet());

		LiveDashboard.INSTANCE.setRobotX(drivetrain.getLocalization().getRobotPosition().getTranslation().getX().getFeet());
		LiveDashboard.INSTANCE.setRobotY(drivetrain.getLocalization().getRobotPosition().getTranslation().getY().getFeet());
		LiveDashboard.INSTANCE.setRobotHeading(drivetrain.getLocalization().getRobotPosition().getRotation().getRadian());

		SmartDashboard.putNumber("Left talon speed", drivetrain.getLeft().getFeetPerSecond());
		SmartDashboard.putNumber("Left talon error", drivetrain.getLeft().getClosedLoopError().getFeet());
		SmartDashboard.putNumber("Right talon speed", drivetrain.getRight().getFeetPerSecond());
		SmartDashboard.putNumber("Right talon error", drivetrain.getRight().getClosedLoopError().getFeet());

		List<Double> feetPerSecond = Arrays.asList(
				VelocityKt.getFeetPerSecond(drivetrain.getLeft().getVelocity()),
				VelocityKt.getFeetPerSecond(drivetrain.getRight().getVelocity()));
		List<Double> feetPerSecondPerSecond = Arrays.asList(
				(VelocityKt.getFeetPerSecond(drivetrain.getLeft().getVelocity()) - drivetrain.lastFeetPerSecond.get(0)) / 0.02d,
				(VelocityKt.getFeetPerSecond(drivetrain.getRight().getVelocity()) - drivetrain.lastFeetPerSecond.get(0)) / 0.02d);
		SmartDashboard.putNumber("Left drivetrian feet per second", feetPerSecond.get(0));
		SmartDashboard.putNumber("Right drivetrian feet per second", feetPerSecond.get(1));

		SmartDashboard.putNumber("7 feet per second is", drivetrain.getLeft().getModel().toNativeUnitPosition(LengthKt.getFeet(7)).getValue());

		SmartDashboard.putNumber("Current Gyro angle", drivetrain.getGyro());

		// Limelight stuff
		// double[] limelightdata = limelight.getData();

		// SmartDashboard.putNumber("Vision targets?", limelightdata[0]);
		// SmartDashboard.putNumber("Horizontal offset", limelightdata[1]);
		// SmartDashboard.putNumber("Vertical offset", limelightdata[2]);
		// SmartDashboard.putNumber("Target area", limelightdata[3]);
		// SmartDashboard.putNumber("Target skew", limelightdata[4]);
		// SmartDashboard.putNumber("Vision pipeline latency", limelightdata[5]);

		// long elapsed = System.currentTimeMillis() - now;
		// System.out.println("RobotPeriodic took " + elapsed + "ms");
	}

}
