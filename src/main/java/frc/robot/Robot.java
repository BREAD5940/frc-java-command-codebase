package frc.robot;

import org.ghrobotics.lib.debug.LiveDashboard;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2d;

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
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.subsystems.Intake;
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
	public static Intake intake = Intake.getInstance();
	public static DriveTrain drivetrain = DriveTrain.getInstance();
	public static SuperStructure superstructure = SuperStructure.getInstance();
	public static LimeLight limelight = new LimeLight();
	public static DoubleSolenoid shifterDoubleSolenoid;
	public static DoubleSolenoid intakeDoubleSolenoid;
	public static DoubleSolenoid elevatorShifterDoubleSolenoid;
	public static AutoMotion m_auto;
	SendableChooser<Command> m_chooser = new SendableChooser<Command>();
	public static Compressor compressor = new Compressor(9);

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
		boolean reset = false;
		if (superstructure.getElbow().getMaster().getSensorCollection().isFwdLimitSwitchClosed()) {
			RoundRotation2d new_ = RoundRotation2d.getDegree(15);
			superstructure.getElbow().getMaster().setSensorPosition(RoundRotation2d.getDegree(15));
			System.out.println("elbow fwd triggered! new pos: " + new_.getDegree());
			reset = true;
		}
		if (superstructure.getWrist().getMaster().getSensorCollection().isFwdLimitSwitchClosed()) {
			System.out.println("wrist fwd triggered!");
			superstructure.getWrist().getMaster().setSensorPosition(RoundRotation2d.getDegree(90));
			reset = true;
		}
		if (superstructure.getElbow().getMaster().getSensorCollection().isRevLimitSwitchClosed()) {
			System.out.println("elbow rev triggered!");
			superstructure.getElbow().getMaster().setSensorPosition(RoundRotation2d.getDegree(-180 - 15));
			reset = true;
		}
		if (superstructure.getWrist().getMaster().getSensorCollection().isRevLimitSwitchClosed()) {
			System.out.println("wrist rev triggered!");
			superstructure.getWrist().getMaster().setSensorPosition(RoundRotation2d.getDegree(-90));
			reset = true;
		}
		if (reset) {
			superstructure.getCurrentCommand().cancel();
			superstructure.getDefaultCommand().start();
		}

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

		LiveDashboard.INSTANCE.setRobotX(drivetrain.getLocalization().getRobotPosition().getTranslation().getX().getFeet());
		LiveDashboard.INSTANCE.setRobotY(drivetrain.getLocalization().getRobotPosition().getTranslation().getY().getFeet());
		LiveDashboard.INSTANCE.setRobotHeading(drivetrain.getLocalization().getRobotPosition().getRotation().getRadian());

		SmartDashboard.putNumber("Current Gyro angle", drivetrain.getGyro());

		SmartDashboard.putData(drivetrain);
		SmartDashboard.putNumber("Current elbow angle: ", SuperStructure.getInstance().getElbow().getMaster().getSensorPosition().getDegree());
		SmartDashboard.putNumber("Current wrist angle: ", SuperStructure.getInstance().getWrist().getMaster().getSensorPosition().getDegree());
		SmartDashboard.putData(superstructure);

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

		// if (getElbow().getMaster().getSensorCollection().isFwdLimitSwitchClosed())
		// System.out.println("elbow limit: " + superstructure.getElbow().getMaster().getSensorCollection().isFwdLimitSwitchClosed());
		// if (getWrist().getMaster().getSensorCollection().isFwdLimitSwitchClosed())
		// System.out.println("wrist limit: " + superstructure.getWrist().getMaster().getSensorCollection().isFwdLimitSwitchClosed());

	}

}
