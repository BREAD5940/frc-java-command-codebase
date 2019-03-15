package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import org.ghrobotics.lib.localization.Localization;
import org.ghrobotics.lib.localization.TankEncoderLocalization;
import org.ghrobotics.lib.mathematics.twodim.control.FeedForwardTracker;
import org.ghrobotics.lib.mathematics.twodim.control.PurePursuitTracker;
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker;
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.subsystems.drive.DifferentialTrackerDriveBase;
import org.ghrobotics.lib.wrappers.ctre.FalconSRX;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import com.team254.lib.physics.DifferentialDrive;
import com.team254.lib.physics.DifferentialDrive.ChassisState;
import com.team254.lib.physics.DifferentialDrive.WheelState;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.commands.subsystems.drivetrain.ArcadeDrive;
import frc.robot.commands.subsystems.drivetrain.SetGearCommand;
import frc.robot.commands.subsystems.drivetrain.TrajectoryTrackerCommand;
import frc.robot.lib.enums.TransmissionSide;
import frc.robot.lib.motion.Util;
import frc.robot.lib.obj.DriveSignal;
import kotlin.ranges.RangesKt;

// import frc.robot.commands.drivetrain_shift_high;
// import frc.robot.commands.drivetrain_shift_low;

/**
 * Drivetrain subsystem. Initilizes the 4 drivetrain talons based on robotconfig
 * settings. Also, some motion profile stuff (which is hopefully depricated).
 * Contains methods for setting PID values, shifting, getting drivetrain encoder
 * status, setting speeds and voltages, and the arcade drive method.
 * 
 * @author Matthew Morley
 */
public class DriveTrain extends Subsystem implements DifferentialTrackerDriveBase {

	private static DriveTrain instance;

	public List<Double> lastCommandedVoltages;
	public List<Double> lastFeetPerSecond = Arrays.asList(0d, 0d);

	public AHRS gyro = new AHRS(SPI.Port.kMXP);
	double gyroZero;

	private Localization localization;

	public Localization getLocalization() {
		return localization;
	}

	private static double kQuickStopThreshold = 0.2;// DifferentialDrive.kDefaultQuickStopThreshold;
	private static double kQuickStopAlpha = 0.1;// DifferentialDrive.kDefaultQuickStopAlpha;
	private double quickStopAccumulator = 0;

	private RamseteTracker ramseteTracker;
	private FeedForwardTracker feedForwardTracker;
	private PurePursuitTracker purePursuitTracker;

	public static enum Gear {
		LOW, HIGH;
	}

	private Gear mCurrentGear;

	public Gear getCachedGear() {
		return mCurrentGear;
	}

	Notifier localizationNotifier; // TODO maybe make me do stuff?

	public static enum TrajectoryTrackerMode {
		RAMSETE, PUREPURSUIT, FEEDFORWARD, PID
	}

	private TrajectoryTrackerMode trackerMode = TrajectoryTrackerMode.RAMSETE;

	public void setTrackerMode(TrajectoryTrackerMode mode) {
		trackerMode = mode;
	}

	// private DCMotorTransmission mLeftTransmissionModel, mRightTransmissionModel;
	private DifferentialDrive lowGearDifferentialDrive, highGearDifferentialDrive;
	private Transmission leftTransmission, rightTransmission;

	private TrajectoryTrackerMode kDefaulTrajectoryTrackerMode = TrajectoryTrackerMode.RAMSETE;

	private DriveTrain() {
		leftTransmission = new Transmission(RobotConfig.driveTrain.leftTalons.m_left_talon_port,
				RobotConfig.driveTrain.leftTalons.s_left_talon_port, Transmission.EncoderMode.CTRE_MagEncoder_Relative,
				TransmissionSide.LEFT, true);
		rightTransmission = new Transmission(RobotConfig.driveTrain.rightTalons.m_right_talon_port,
				RobotConfig.driveTrain.rightTalons.s_right_talon_port, Transmission.EncoderMode.CTRE_MagEncoder_Relative,
				TransmissionSide.RIGHT, false);

		/* Create a localization object because lamda expressions are fun */
		localization = new TankEncoderLocalization(() -> Rotation2dKt.getDegree(getGyro(true)),
				() -> getLeft().getDistance(), () -> getRight().getDistance());
		/* set the robot pose to 0,0,0 */
		localization.reset(new Pose2d());

		lowGearDifferentialDrive = Constants.kHighGearDifferentialDrive;
		highGearDifferentialDrive = Constants.kLowGearDifferentialDrive;

		ramseteTracker = new RamseteTracker(Constants.kDriveBeta, Constants.kDriveZeta);
		purePursuitTracker = new PurePursuitTracker(Constants.kLat, Constants.kLookaheadTime,
				Constants.kMinLookaheadDistance);
		feedForwardTracker = new FeedForwardTracker();

		localizationNotifier = new Notifier(() -> {
			this.getLocalization().update();
		});
		localizationNotifier.startPeriodic(0.01);
	}

	public DifferentialDrive getDifferentialDrive() {
		// String mes = (mCurrentGear == Gear.LOW) ? "lowGearDifferentialDrive" : "highGearDifferentialDrive"; 
		SmartDashboard.putString("current drive model", (mCurrentGear == Gear.LOW) ? "lowGearDifferentialDrive" : "highGearDifferentialDrive");
		return (mCurrentGear == Gear.LOW) ? lowGearDifferentialDrive : highGearDifferentialDrive;
	}

	// public double mCurrentSolenoidValue = Robot.shifterso

	// public DCMotorTransmission getTransmissionModel() {
	//   return mTransmissionModel;
	// }

	public FalconSRX<Length> getLeftMotor() {
		return getLeft().getMaster();
	}

	public FalconSRX<Length> getRightMotor() {
		return getRight().getMaster();
	}

	public synchronized static DriveTrain getInstance() {
		if (instance == null)
			instance = new DriveTrain();
		return instance;
	}

	// public WPI_TalonSRX getLeftMotor() {
	// return m_left_talon;
	// }

	public RamseteTracker getRamseteTracker() {
		return ramseteTracker;
	}

	public TrajectoryTracker getTrajectoryTracker() {
		return getTrajectoryTracker(kDefaulTrajectoryTrackerMode);
	}

	public TrajectoryTracker getTrajectoryTracker(TrajectoryTrackerMode mode) {
		switch (mode) {
		case RAMSETE:
			return ramseteTracker;
		case FEEDFORWARD:
			return feedForwardTracker;
		case PUREPURSUIT:
			return purePursuitTracker;
		case PID:
			return null;
		default:
			return ramseteTracker;
		}
	}

	public void init() {
		zeroEncoders();
		setHighGear();
	}

	public void setHighGear() {
		leftTransmission.setClosedLoopGains(RobotConfig.driveTrain.leftTalons.velocity_kp_high,
				RobotConfig.driveTrain.leftTalons.velocity_ki_high, RobotConfig.driveTrain.leftTalons.velocity_kd_high,
				RobotConfig.driveTrain.leftTalons.velocity_kf_high, RobotConfig.driveTrain.leftTalons.velocity_izone_high,
				RobotConfig.driveTrain.leftTalons.velocity_max_integral_high);
		rightTransmission.setClosedLoopGains(RobotConfig.driveTrain.rightTalons.velocity_kp_high,
				RobotConfig.driveTrain.rightTalons.velocity_ki_high, RobotConfig.driveTrain.rightTalons.velocity_kd_high,
				RobotConfig.driveTrain.rightTalons.velocity_kf_high, RobotConfig.driveTrain.rightTalons.velocity_izone_high,
				RobotConfig.driveTrain.rightTalons.velocity_max_integral_high);
		// Trigger solenoids
		Robot.drivetrain_shift_high();
		mCurrentGear = Gear.HIGH;
	}

	public void setLowGear() {
		leftTransmission.setClosedLoopGains(RobotConfig.driveTrain.leftTalons.velocity_kp_low,
				RobotConfig.driveTrain.leftTalons.velocity_ki_low, RobotConfig.driveTrain.leftTalons.velocity_kd_low,
				RobotConfig.driveTrain.leftTalons.velocity_kf_low, RobotConfig.driveTrain.leftTalons.velocity_izone_low,
				RobotConfig.driveTrain.leftTalons.velocity_max_integral_low);
		rightTransmission.setClosedLoopGains(RobotConfig.driveTrain.rightTalons.velocity_kp_low,
				RobotConfig.driveTrain.rightTalons.velocity_ki_low, RobotConfig.driveTrain.rightTalons.velocity_kd_low,
				RobotConfig.driveTrain.rightTalons.velocity_kf_low, RobotConfig.driveTrain.rightTalons.velocity_izone_low,
				RobotConfig.driveTrain.rightTalons.velocity_max_integral_low);
		// Trigger solenoids
		Robot.drivetrain_shift_low();
		mCurrentGear = Gear.LOW;
	}

	public void setGear(Gear gear) {
		switch (gear) {
		case HIGH:
			setHighGear();
			break;
		case LOW:
			setLowGear();
			break;
		default:
			break;
		}
	}

	public void setNeutralMode(NeutralMode mode) {
		getLeft().getMaster().setNeutralMode(mode);
		getRight().getMaster().setNeutralMode(mode);
	}

	public Transmission getLeft() {
		return leftTransmission;
	}

	public Transmission getRight() {
		return rightTransmission;
	}

	public void zeroEncoders() {
		leftTransmission.zeroEncoder();
		rightTransmission.zeroEncoder();
	}

	/**
	 * Stop the transmissions by zeroing their outputs
	 */
	public void stop() {
		getLeft().stop();
		getRight().stop();
	}

	// /**
	// * Set the drivetrain speeds in feet per second
	// * @param mleftSpeed in feet per second
	// * @param mRightSpeed in feet per second
	// */
	// public void setFeetPerSecond(double mleftSpeed, double mRightSpeed) {
	// setSpeeds(
	// EncoderLib.distanceToRaw(
	// mleftSpeed,
	// RobotConfig.driveTrain.left_wheel_effective_diameter / 12,
	// RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION) / 10,
	// EncoderLib.distanceToRaw(
	// mRightSpeed,
	// RobotConfig.driveTrain.left_wheel_effective_diameter / 12,
	// RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION) / 10
	// );
	// }

	/**
	 * An even more lazy version of setSpeeds This will literally set the voltage of
	 * the left and right talons (from -12 to 12 ofc per battery voltage)
	 * 
	 * @param left_voltage
	 * @param right_voltage
	 */
	public void setVoltages(double left_voltage, double right_voltage) {
		getLeft().getMaster().set(ControlMode.PercentOutput, left_voltage / 12);
		getRight().getMaster().set(ControlMode.PercentOutput, right_voltage / 12);
	}

	public void setClosedLoop(DriveSignal signal) {
		setCLosedLoop(signal.getLeft(), signal.getRight(), signal.getLeftPercent(), signal.getRightPercent(),
				signal.getBrakeMode());
	}

	/**
	 * Set the drivetrain talons to closed loop velocity mode, given a
	 * Velocity<Length> object to represent a unit-signed speed for the left and
	 * right spides.
	 * 
	 * @param left  velocity in a Velocity<Length> Falconlib object
	 * @param right velocity in a Velocity<Length> Falconlib object
	 */
	public void setCLosedLoop(Velocity<Length> left, Velocity<Length> right, double leftPercent, double rightPercent,
			boolean brakeMode) {
		setNeutralMode((brakeMode) ? NeutralMode.Brake : NeutralMode.Coast);
		getLeft().getMaster().set(ControlMode.Velocity, left, DemandType.ArbitraryFeedForward, leftPercent);
		getRight().getMaster().set(ControlMode.Velocity, right, DemandType.ArbitraryFeedForward, rightPercent);
	}

	public void setClosedLoop(Velocity<Length> left, Velocity<Length> right) {
		setCLosedLoop(left, right, 0, 0, false);
	}

	/**
	 * Set the raw speeds of the talons. Use setClosedLoop() instead.
	 * 
	 * @deprecated
	 */
	@Deprecated
	public void setRawSpeeds(double leftRaw, double rightRaw) {
		getLeftMotor().set(ControlMode.Velocity, leftRaw);
		getRightMotor().set(ControlMode.Velocity, rightRaw);
	}

	/**
	 * setPowers is an even more lazy version of set speeds. This will literally set
	 * the throttle of the left and right talons (from -1 to 1 ofc, like normal)
	 * 
	 * @param left_power
	 * @param right_power
	 */
	public void setPowers(double left_power, double right_power) {
		leftTransmission.getMaster().set(ControlMode.PercentOutput, left_power);
		rightTransmission.getMaster().set(ControlMode.PercentOutput, right_power);
	}

	public Pose2d getRobotPosition() {
		return getLocalization().getRobotPosition();
	}

	public void setRobotPosition(Pose2d pose2d) {
		getLocalization().reset(pose2d);
	}

	public void arcadeDrive(double linear, double rotation) {
		arcadeDrive(linear, rotation, true);
	}

	public void arcadeDrive(double linearPercent, double rotationPercent, boolean squareInputs) {
		linearPercent = Util.limit(linearPercent, 1);
		linearPercent = Util.deadband(linearPercent, 0.07);

		rotationPercent = Util.limit(rotationPercent, 1);
		rotationPercent = Util.deadband(rotationPercent, 0.07);

		// Square the inputs (while preserving the sign) to increase fine control
		// while permitting full power.
		if (squareInputs) {
			linearPercent = Math.copySign(linearPercent * linearPercent, linearPercent);
			rotationPercent = Math.copySign(rotationPercent * rotationPercent, rotationPercent);
		}

		double leftMotorOutput;
		double rightMotorOutput;

		double maxInput = Math.copySign(Math.max(Math.abs(linearPercent), Math.abs(rotationPercent)), linearPercent);

		if (linearPercent >= 0.0) {
			// First quadrant, else second quadrant
			if (rotationPercent >= 0.0) {
				leftMotorOutput = maxInput;
				rightMotorOutput = linearPercent - rotationPercent;
			} else {
				leftMotorOutput = linearPercent + rotationPercent;
				rightMotorOutput = maxInput;
			}
		} else {
			// Third quadrant, else fourth quadrant
			if (rotationPercent >= 0.0) {
				leftMotorOutput = linearPercent + rotationPercent;
				rightMotorOutput = maxInput;
			} else {
				leftMotorOutput = maxInput;
				rightMotorOutput = linearPercent - rotationPercent;
			}
		}
		// Logger.log("Linear input " + linearPercent + " turn input " +
		// rotationPercent);
		// Logger.log("left motor output " + leftMotorOutput + " right motor output " +
		// rightMotorOutput);

		var isHighGear = getCachedGear() == Gear.HIGH;

		final double lowGearForward = Util.toMeters(5);
		final double lowGearTurn = Util.toMeters(6);
		final double highGearForward = Util.toMeters(10);
		final double highGearTurn = Util.toMeters(6);

		double forwardSpeed = linearPercent * ((isHighGear) ? highGearForward : lowGearForward);
		double turnSpeed = -1 * rotationPercent * ((isHighGear) ? highGearTurn : lowGearTurn);

		ChassisState mVelocity = new ChassisState(forwardSpeed, turnSpeed);
		if(isFirstRun) {
			mCachedChassisState = new ChassisState();
			isFirstRun = false;
		}
		
		ChassisState mAcceleration = new ChassisState(
			mVelocity.getLinear() - mCachedChassisState.getLinear(), 
			mVelocity.getAngular() - mCachedChassisState.getAngular()
		);

		// var kinematics = getDifferentialDrive();
		// mAcceleration = new ChassisState(
		// 	Math.max(mAcceleration.getLinear(),
		// 		kinematics.
		// 	))
		// );

		this.setOutputFromDynamics(mVelocity, mAcceleration);
		
		
		mCachedChassisState = mVelocity;

		// tankDrive(left/12, right/12);

		// tankDrive(leftMotorOutput, rightMotorOutput);
		// tankDrive(0.2, 0.2);
	}

	private ChassisState mCachedChassisState;
	public boolean isFirstRun = true;

	public void curvatureDrive(double linearPercent, double curvaturePercent, boolean isQuickTurn) {
		double angularPower;
		boolean overPower;

		if (isQuickTurn) {
			if (Math.abs(linearPercent) < kQuickStopThreshold) {
				quickStopAccumulator = (1 - kQuickStopAlpha) * quickStopAccumulator
						+ kQuickStopAlpha * RangesKt.coerceIn(curvaturePercent, -1, 1) * 2.0;
			}

			overPower = true;
			angularPower = curvaturePercent;
		} else {
			overPower = false;
			angularPower = Math.abs(linearPercent) * curvaturePercent - quickStopAccumulator;

			if (quickStopAccumulator > 1) {
				quickStopAccumulator -= 1;
			} else if (quickStopAccumulator < -1) {
				quickStopAccumulator += 1;
			} else {
				quickStopAccumulator = 0;
			}
		}

		double leftMotorOutput = linearPercent + angularPower;
		double rightMotorOutput = linearPercent - angularPower;

		// If rotation is overpowered, reduce both outputs to within acceptable range
		if (overPower) {
			if (leftMotorOutput > 1.0) {
				rightMotorOutput -= leftMotorOutput - 1.0;
				leftMotorOutput = 1.0;
			} else if (rightMotorOutput > 1.0) {
				leftMotorOutput -= rightMotorOutput - 1.0;
				rightMotorOutput = 1.0;
			} else if (leftMotorOutput < -1.0) {
				rightMotorOutput -= leftMotorOutput + 1.0;
				leftMotorOutput = -1.0;
			} else if (rightMotorOutput < -1.0) {
				leftMotorOutput -= rightMotorOutput + 1.0;
				rightMotorOutput = -1.0;
			}
		}

		// Normalize the wheel speeds
		double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
		if (maxMagnitude > 1.0) {
			leftMotorOutput /= maxMagnitude;
			rightMotorOutput /= maxMagnitude;
		}

		tankDrive(leftMotorOutput, rightMotorOutput);
	}

	public void tankDrive(double leftPercent, double rightPercent) {
		lastCommandedVoltages = Arrays.asList(leftPercent * 12, rightPercent * 12);

		// Logger.log("Left error: " + getLeft().getClosedLoopError().getFeet());
		// Logger.log("Right error: " + getRight().getClosedLoopError().getFeet());

		// Logger.log("Left speed: " +
		// getLeft().getMaster().getSensorVelocity().getValue());
		// Logger.log("Right speed: " + getRight().getVelocity().getValue());

		if (leftPercent < 0.06 && leftPercent > -0.06)
			leftPercent = 0.0;

		if (rightPercent < 0.06 && rightPercent > -0.06)
			rightPercent = 0.0;

		getLeft().getMaster().set(ControlMode.PercentOutput, leftPercent); // because C O M P E N S A T I O N
		getRight().getMaster().set(ControlMode.PercentOutput, rightPercent);

		// 2.1 meters per second in low gear
		//

		// getLeft().getMaster().set(ControlMode.Velocity, LengthKt.getFeet(leftPercent
		// * 8)); // because C O M P E N S A T I O N
		// getRight().getMaster().set(ControlMode.Velocity,
		// LengthKt.getFeet(rightPercent * 8));
	}

	/**
	 * Get the default trajectory following command given a path to follow.
	 * 
	 * @param trajectory the path
	 * @return a trajectory follower command to follow it
	 */
	public TrajectoryTrackerCommand followTrajectory(TimedTrajectory<Pose2dWithCurvature> trajectory) {
		return followTrajectory(trajectory, false);
	}

	/**
	 * Get a command to follow a trajectory given a path and an option to reset
	 * odometry
	 * 
	 * @param trajectory to follow
	 * @param reset      if we should reset robot odometry to the initial pose or
	 *                   not
	 */
	public TrajectoryTrackerCommand followTrajectory(TimedTrajectory<Pose2dWithCurvature> trajectory, Gear gear, boolean reset) {
		mCurrentGear = Robot.getDrivetrainGear();
		return new TrajectoryTrackerCommand(this, () -> trajectory, reset);
	}

	public TrajectoryTrackerCommand followTrajectory(TimedTrajectory<Pose2dWithCurvature> trajectory, boolean reset) {
		return followTrajectory(trajectory, Robot.getDrivetrainGear(), reset);
	}

	public Command followTrajectory(TimedTrajectory<Pose2dWithCurvature> trajectory,
			TrajectoryTrackerMode mode, boolean reset) {
		// kDefaulTrajectoryTrackerMode = mode;
		mCurrentGear = Robot.getDrivetrainGear();
		return new TrajectoryTrackerCommand(this, getTrajectoryTracker(mode), () -> trajectory, reset);
	}

	/**
	 * Get a CommandGroup that will follow a supplied trajectory with a given traker mode, Gear and if we should reset robot pose or not
	 * @param trajectory that the robot should follow
	 * @param mode of the tracker (e.g. Ramsete)
	 * @param gear that the action should be in
	 * @param resetPose if we should reset the robot pose or not
	 */
	public Command followTrajectoryWithGear(TimedTrajectory<Pose2dWithCurvature> trajectory,
			TrajectoryTrackerMode mode, Gear gear, boolean resetPose) {
		mCurrentGear = gear;
		CommandGroup mCommandGroup = new CommandGroup();
		mCommandGroup.addParallel(new SetGearCommand(gear));
		mCommandGroup.addSequential(followTrajectory(trajectory, mode, resetPose));
		return mCommandGroup;
	}

	/**
	 * Get the angle of the gyro, accounting for the gyro zero angle
	 * 
	 * @return compensated gyro angle
	 */
	public double getGyro() {
		return gyro.getAngle() - gyroZero;
	}

	/**
	 * Return the angle of the gyro. If the argument returns true the output will be
	 * inverted (positive numbers is counter clockwise)
	 */
	public double getGyro(boolean inverted) {
		double gyroang;
		if (inverted) {
			gyroang = getGyro() * (-1);
		} else {
			gyroang = getGyro();
		}
		// Logger.log("Gyroangle: " + gyroang);
		return gyroang;
	}

	public void zeroGyro() {
		gyroZero = gyro.getAngle();
	}

	@Override
	public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		setDefaultCommand(new ArcadeDrive());
		// setDefaultCommand(new auto_action_DRIVE(5, "high", 5, 30));
	}

}
