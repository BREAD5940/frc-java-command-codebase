package frc.robot;

import org.ghrobotics.lib.debug.LiveDashboard;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.team5940.pantry.exparimental.command.SendableCommandBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auto.AutoMotion;
import frc.robot.commands.auto.TerribleAutoChooser;
import frc.robot.commands.auto.Trajectories;
import frc.robot.commands.subsystems.superstructure.ZeroElevatorDisabled;
import frc.robot.lib.Logger;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.lib.statemachines.AutoMotionStateMachine;
import frc.robot.lib.statemachines.AutoMotionStateMachine.GoalHeight;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.LimeLight.LEDMode;
import frc.robot.subsystems.superstructure.SuperStructure;

/**
 * Main robot class. There shouldn't be a *ton* of stuff here, mostly init
 * functions and smartdashboard stuff.
 *
 * @author Matthew Morley
 */
public class Robot extends TimedRobot {
	public static SendableChooser<GoalHeight> mGh;
	public static OI m_oi;
	public static AutoMotionStateMachine autoState; //TODO this should be static, right?
	// public static Intake intake = new Intake();
	// public static Elevator elevator = new Elevator();
	public static DriveTrain drivetrain = DriveTrain.getInstance();
	public static SuperStructure superstructure = SuperStructure.getInstance();
	// public static VisionProcessor visionProcessor = new VisionProcessor();
	public static LimeLight limelight = LimeLight.getInstance();
	// public static LIDARSubsystem lidarSubsystem = new LIDARSubsystem();
	public static DoubleSolenoid shifterDoubleSolenoid;
	public static DoubleSolenoid intakeDoubleSolenoid;
	public static DoubleSolenoid elevatorShifterDoubleSolenoid;
	public static AutoMotion m_auto;
	// SendableChooser<Command> m_chooser = new SendableChooser<Command>();
	TerribleAutoChooser mAutoChooser;
	public static Compressor compressor = new Compressor(9);

	private Notifier mResetNotifier;

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
			shifterDoubleSolenoid = new DoubleSolenoid(9, 4, 5);
		return shifterDoubleSolenoid;
	}

	public static DoubleSolenoid getIntakeSolenoidInstance() {
		if (intakeDoubleSolenoid == null)
			intakeDoubleSolenoid = new DoubleSolenoid(9, 0, 1);
		return intakeDoubleSolenoid;
	}

	public static DoubleSolenoid getElevatorShifter() {
		if (elevatorShifterDoubleSolenoid == null) {
			elevatorShifterDoubleSolenoid = new DoubleSolenoid(9, 2, 3);
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
			elevatorShifterDoubleSolenoid = new DoubleSolenoid(9, 2, 3);
		if (value == null)
			value = DoubleSolenoid.Value.kForward; // TODO hack
		elevatorShifterDoubleSolenoid.set(value); // FIXME it's a hack
	}

	public static double mPeriod = 0.025d;

	/**
	* FIXME it's a hack, set the period to 25ms
	*/
	public Robot() {
		super(mPeriod);
	}

	public static enum RobotState {
		AUTO(1), TELEOP(2), DISABLED(0), TEST(3);

		final int value;

		private RobotState(int value) {
			this.value = value;
		}
	}

	public static RobotState getState() {
		var ds = DriverStation.getInstance();
		if (ds.isAutonomous())
			return RobotState.AUTO;
		else if (ds.isOperatorControl())
			return RobotState.TELEOP;
		else if (ds.isTest())
			return RobotState.TEST;
		else
			return RobotState.DISABLED;
	}

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {

		Logger.clearLog();

		SmartDashboard.putData(Scheduler.getInstance()); //it'll let you see all the active commands and (I think) cancel them too

		SmartDashboard.putData(zeroElevatorWhileDisabled);

		mAutoChooser = new TerribleAutoChooser();
		mAutoChooser.addOptions();
		System.out.println("Auto chooser sent!");
		Trajectories.generateAllTrajectories();

		Shuffleboard.getTab("Auto").getLayout("Path selection", BuiltInLayouts.kList).add(mAutoChooser.getChooser()).withSize(2, 5).withPosition(0, 0);

		if (drivetrain == null)
			drivetrain = DriveTrain.getInstance();
		// FIXME Jocelyn this might mess with auto stuff, will it? (I think no?)
		drivetrain.getLocalization().reset(new Pose2d(LengthKt.getFeet(5.5), LengthKt.getFeet(17), new Rotation2d(0f, 0f, false)));

		autoState = new AutoMotionStateMachine();
		// logger = Logger.getInstance();

		mGh = new SendableChooser<GoalHeight>();
		mGh.setDefaultOption("Low", GoalHeight.LOW);
		mGh.addOption("Middle", GoalHeight.MIDDLE);
		mGh.addOption("High", GoalHeight.HIGH);
		SmartDashboard.putData("Goal Height", mGh);

		SmartDashboard.putData("LIMELIGHT LED ON", new LimeLight.SetLEDs(LEDMode.kON));
		SmartDashboard.putData("LIMELIGHT LED OFF", new LimeLight.SetLEDs(LEDMode.kOFF));

		m_oi = new OI();

		// SmartDashboard.putData(Intake.getInstance());
		// SmartDashboard.putData(shifterDoubleSolenoid);

		compressor.setClosedLoopControl(true);

		drivetrain.init();
		// elevator.init();
		// wrist.init();
		drivetrain.zeroGyro();
		var elevator = SuperStructure.getElevator();
		var startingHeightTicks = elevator.getModel().toNativeUnitPosition(LengthKt.getInch(24)).getValue();
		// 600 is the boiiii
		var target_ = 650;
		var target_COMP = 650;
		var tickkkkks_ = (SuperStructure.getElevator().getMaster().getSensorCollection().getPulseWidthPosition() % 2048) * ((SuperStructure.getElevator().getMaster().getSensorCollection().getPulseWidthPosition() > 0) ? 1 : -1);
		var delta_ = (tickkkkks_ - (int) target_COMP) * -1;

		// elevator.getMaster().setSelectedSensorPosition((int) (startingHeightTicks + delta_));
		// elevator.getMaster().setSelectedSensorPosition((int) (startingHeightTicks));

		var proximal = SuperStructure.getInstance().getElbow();
		// var startingAngleTicks = (int) proximal.getMaster().getTicks(RoundRotation2d.getDegree(-90)) + (-640) + (proximal.getMaster().getSensorCollection().getPulseWidthPosition() % 2048 * Math.signum(proximal.getMaster().getSensorCollection().getPulseWidthPosition() % 2048));
		var tickkkkks = (superstructure.getElbow().getMaster().getSensorCollection().getPulseWidthPosition() % 2048) * ((superstructure.getElbow().getMaster().getSensorCollection().getPulseWidthPosition() > 0) ? 1 : -1);
		var targetProximal_ = 1400;
		var targetProximal_COMP = 1900;
		var delta = (tickkkkks - (int) targetProximal_COMP) * -1;
		var startingAngleTicks = proximal.getMaster().getTicks(RoundRotation2d.getDegree(-78));

		proximal.getMaster().setSelectedSensorPosition((int) (0 + startingAngleTicks));
		// proximal.getMaster().setSelectedSensorPosition((int) (startingAngleTicks));

		var wrist = SuperStructure.getInstance().getWrist();
		var wristStart = (int) wrist.getMaster().getTicks(RoundRotation2d.getDegree(-43 + 4 - 9));
		var targetWrist = (int) 1000;
		var targetWristComp = 1500 + 150;
		var correctionDelta = (superstructure.getElbow().getMaster().getSensorCollection().getPulseWidthPosition() % 2048) * ((superstructure.getElbow().getMaster().getSensorCollection().getPulseWidthPosition() > 0) ? 1 : -1);
		var deltaW = (correctionDelta - (int) targetWristComp) * 1;

		wrist.getMaster().setSelectedSensorPosition((int) (deltaW + wristStart));
		// wrist.getMaster().setSelectedSensorPosition((int) (wristStart));

		elevator.getMaster().setSelectedSensorPosition((int) (elevator.getModel().toNativeUnitPosition(LengthKt.getInch(24)).getValue()), 0, 0); // just to be super sure the elevator is safe-ishhhh
		var cmd = zeroElevatorWhileDisabled;
		cmd.schedule();

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
		//		SmartDashboard.putData("Zero elevator height:", new ZeroSuperStructure("elevator"));
		//		SmartDashboard.putData("Zero elbow angle:", new ZeroSuperStructure("elbow"));
		//		SmartDashboard.putData("Zero wrist angle:", new ZeroSuperStructure("wrist"));
		//		SmartDashboard.putData("Max elevator height:", new ZeroSuperStructure("maxElevator"));
		//		// SmartDashboard.putData("Top of inner stage elevator height", new ZeroSuperStructure("topInnerElevator"));
		//		SmartDashboard.putData("Max wrist angle:", new ZeroSuperStructure("maxWrist"));
		//		SmartDashboard.putData("Min wrist angle:", new ZeroSuperStructure("minWrist"));
		//		SmartDashboard.putData("Max elbow angle:", new ZeroSuperStructure("maxElbow"));
		//		SmartDashboard.putData("Min elbow angle:", new ZeroSuperStructure("minElbow"));
		drivetrain.zeroEncoders();
		System.out.println("Robot init'ed and encoders zeroed!");

		mResetNotifier = new Notifier(() -> {

			// SuperStructure.getElevator().getMaster().setSelectedSensorPosition((int)startingHeightTicks);

			// SuperStructure.getInstance().getElbow().getMaster().setSelectedSensorPosition(proximal.getMaster().getTicks(RoundRotation2d.getDegree(-94)));

			// SuperStructure.getInstance().getWrist().getMaster().setSelectedSensorPosition(proximal.getMaster().getTicks(RoundRotation2d.getDegree(-42)));

			boolean reset = false;
			if (superstructure.getElbow().getMaster().getSensorCollection().isFwdLimitSwitchClosed()) {
				// RoundRotation2d new_ = RoundRotation2d.getDegree(15);
				// superstructure.getElbow().getMaster().setSensorPosition(RoundRotation2d.getDegree(15));
				// System.out.println("elbow fwd triggered! new pos: " + new_.getDegree());
				// reset = true;
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
				superstructure.getDefaultCommand().schedule();
			}

		});

		System.out.println("Welcome to Team 5940 BREAD's 2019 Robot, CROISSANT");

		System.out.println("\n" +
				"                            ***********************                             \n" +
				"                        *******************************                         \n" +
				"                      *******+                   ?*******                       \n" +
				"                   ******                             ******                    \n" +
				"                :********                             ********:                 \n" +
				"             :***********                            :***********~              \n" +
				"           *******    ****                           ****    *******            \n" +
				"         ******        ***                           ***        ******          \n" +
				"       $*****          ****                         ****          ******        \n" +
				"      *****            ****                         ***:            *****       \n" +
				"     ****               ***                         ***               ****      \n" +
				"    ****                ****                       ****                ****     \n" +
				"    ***                  ***                       ***                  ***~    \n" +
				"    ****+                ****                     ****                :*****    \n" +
				"   *******               ****                     ***                *******    \n" +
				"  **** ****               ***+                   ?***               **** ****   \n" +
				" ****   *****             ****                   ****             *****   ****  \n" +
				" ***      ****             ***                   ***             ****      ***  \n" +
				"****       *****           ****                 ****           *****       **** \n" +
				"***:         ****           ***                 ***           ****          *** \n" +
				"***           *****         ****               ****         *****           *** \n" +
				"***            ~****     ~***************************=     ****~            ***:\n" +
				"***              ***** ********************************* *****              *** \n" +
				"***              **********                         **********              *** \n" +
				"****           *********                               *********           **** \n" +
				" ****        ******                                         ******        ****  \n" +
				"  *****  :*******                                             *******   *****   \n" +
				"    ***********                                                 ************    \n" +
				"      *****                                                         *****       \n" +
				"                                                                                \n");

		// mResetNotifier.startPeriodic(0.5);

		//		var frontBack = new CommandGroup();
		//		//		frontBack.addSequential(new ElevatorMove(LengthKt.getInch(24)));
		//		frontBack.addSequential(new SyncedMove(Math.toRadians(-180), true, superstructure));
		//		// frontBack.addSequential(new ArmMove(new IntakeAngle(
		//		// 	new RotatingArmState(RoundRotation2d.getDegree(-210)),
		//		// 	new RotatingArmState(RoundRotation2d.getDegree(-180))
		//		// 	)));
		//
		//		var backFront = new CommandGroup();
		//		//		backFront.addSequential(new ElevatorMove(LengthKt.getInch(24)));
		//		backFront.addSequential(new SyncedMove(Math.toRadians(0), true, superstructure));
		//
		//		SmartDashboard.putData("front to back passthrough", new PassThrough.FrontToBack(superstructure));
		//		SmartDashboard.putData("back to front passthrough", new PassThrough.BackToFront(superstructure));
		//		//		SmartDashboard.putData("THE ONE TRUE PASSTHROUGH", new PassThrough(SuperStructure.getInstance(), () -> SuperStructure.getInstance().getCurrentState().getElbowAngle().getDegree() >= -90));

	}

	public static SendableCommandBase zeroElevatorWhileDisabled = new ZeroElevatorDisabled();

	/**
	 * This function is called once each time the robot enters Disabled mode. You
	 * can use it to reset any subsystem information you want to clear when the
	 * robot is disabled.
	 */
	@Override
	public void disabledInit() {

		SuperStructure.elevator.onDisable();
		SuperStructure.getInstance().getElbow().onDisable();
		SuperStructure.getInstance().getWrist().onDisable();

		// try {
		// mResetNotifier.startPeriodic(0.5);
		// } catch (Exception e) {
		// e.printStackTrace();
		// }

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

	}

	@Override
	public void autonomousInit() {
		mResetNotifier.stop();
		// DriveTrajectoryPathfinder meme = new DriveTrajectoryPathfinder("file");
		// meme.schedule();

		// drivetrain.gyro.reset(); // Reset the current gyro heading to zero
		// drivetrain.zeroEncoders();

		mAutoChooser.getSelection().schedule(); // So this needs a defaut option

		// 	if (RobotConfig.auto.auto_gear == Gear.LOW) {
		// 		drivetrain.setLowGear();
		// 	} else if (RobotConfig.auto.auto_gear == Gear.HIGH) {
		// 		drivetrain.setHighGear();
		// 	} else {
		// 		System.out.println("default auto gear " + RobotConfig.auto.auto_gear + " is not a valid choice!");
		// 	}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {}

	// public void disableTheSuperStructure() {
	// 	superstructure.getElevator().getMaster().configPeakOutputForward(0);
	// 	superstructure.getElevator().getMaster().configPeakOutputReverse(0);

	// 	superstructure.getInstance().getElbow().getMaster().configPeakOutputForward(0);
	// 	superstructure.getInstance().getElbow().getMaster().configPeakOutputReverse(0);

	// 	superstructure.getInstance().getWrist().getMaster().configPeakOutputForward(0);
	// 	superstructure.getInstance().getWrist().getMaster().configPeakOutputReverse(0);
	// }

	@Override
	public void teleopInit() {

		SuperStructure.elevator.onDisable();

		mResetNotifier.stop();
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
		Scheduler.getInstance().run();

		drivetrain.logPeriodicIO();

		// SmartDashboard.putNumber("Limelight estimated distance with angle", LimeLight.getInstance().estimateDistanceFromAngle().getInch());

		// System.out.println("current superstructure state: " + superstructure.getCurrentState());

		// disableTheSuperStructure();

		final boolean postTicks = true;

		if (postTicks) {

			SmartDashboard.putBoolean("Elevator limit switch", SuperStructure.getInstance().getInnerStageMinLimit());

			// var tickkkkks = (int) superstructure.getWrist().getMaster().getTicks(RoundRotation2d.getDegree(-90)) + (-640) + (superstructure.getWrist().getMaster().getSensorCollection().getPulseWidthPosition() % 2048 * Math.signum(superstructure.getWrist().getMaster().getSensorCollection().getPulseWidthPosition() % 2048));
			// var tickkkkks = (superstructure.getElbow().getMaster().getSensorCollection().getPulseWidthPosition() % 2048) * ((superstructure.getElbow().getMaster().getSensorCollection().getPulseWidthPosition() > 0) ? 1 : -1);
			var tickkkkks = (superstructure.getElbow().getMaster().getSensorCollection().getPulseWidthPosition() % 2048) * ((superstructure.getElbow().getMaster().getSensorCollection().getPulseWidthPosition() > 0) ? 1 : -1);
			SmartDashboard.putNumber("Elbow absolute pos ", tickkkkks);

			var tickks = (superstructure.getWrist().getMaster().getSensorCollection().getPulseWidthPosition() % 2048) * ((superstructure.getWrist().getMaster().getSensorCollection().getPulseWidthPosition() > 0) ? 1 : -1);

			SmartDashboard.putNumber("wrist absolute pos ", tickks);

			var ticcccks = (SuperStructure.getElevator().getMaster().getSensorCollection().getPulseWidthPosition() % 2048) * ((SuperStructure.getElevator().getMaster().getSensorCollection().getPulseWidthPosition() > 0) ? 1 : -1);

			SmartDashboard.putNumber("Elevator absolute pos ", tickks);

			// System.out.println(superstructure.getElbow().getMaster().getSensorCollection().getPulseWidthPosition());
			// System.out.println(superstructure.getElbow().getMaster().getSensorPosition().getDegree());

			SmartDashboard.putString(SuperStructure.getInstance().getCurrentState().getCSVHeader(), SuperStructure.getInstance().getCurrentState().toCSV());

		}

		// var elevatorAbsTicks = SuperStructure.getElevator().getMaster().getSensorCollection().getPulseWidthPosition() % 1024;
		// System.out.println(elevatorAbsTicks);

		// System.out.println(String.format("carriage max %s inner stage min %s", superstructure.getCarriageMaxLimit(), superstructure.getInnerStageMinLimit() ));

		SmartDashboard.putNumber("Robot X (feet) ", drivetrain.getLocalization().getRobotPosition().getTranslation().getX().getFeet());
		SmartDashboard.putNumber("Robot Y (feet) ", drivetrain.getLocalization().getRobotPosition().getTranslation().getY().getFeet());

		LiveDashboard.INSTANCE.setRobotX(drivetrain.getLocalization().getRobotPosition().getTranslation().getX().getFeet());
		LiveDashboard.INSTANCE.setRobotY(drivetrain.getLocalization().getRobotPosition().getTranslation().getY().getFeet());
		LiveDashboard.INSTANCE.setRobotHeading(drivetrain.getLocalization().getRobotPosition().getRotation().getRadian());

		// SmartDashboard.putNumber("Left talon speed", drivetrain.getLeft().getFeetPerSecond());
		// SmartDashboard.putNumber("Left talon error", drivetrain.getLeft().getClosedLoopError().getFeet());
		// SmartDashboard.putNumber("Right talon speed", drivetrain.getRight().getFeetPerSecond());
		// SmartDashboard.putNumber("Right talon error", drivetrain.getRight().getClosedLoopError().getFeet());

		// List<Double> feetPerSecond = Arrays.asList(
		// 		VelocityKt.getFeetPerSecond(drivetrain.getLeft().getVelocity()),
		// 		VelocityKt.getFeetPerSecond(drivetrain.getRight().getVelocity()));
		// List<Double> feetPerSecondPerSecond = Arrays.asList(
		// 		(VelocityKt.getFeetPerSecond(drivetrain.getLeft().getVelocity()) - drivetrain.lastFeetPerSecond.get(0)) / 0.02d,
		// 		(VelocityKt.getFeetPerSecond(drivetrain.getRight().getVelocity()) - drivetrain.lastFeetPerSecond.get(0)) / 0.02d);
		// SmartDashboard.putNumber("Left drivetrian feet per second", feetPerSecond.get(0));
		// SmartDashboard.putNumber("Right drivetrian feet per second", feetPerSecond.get(1));

		// SmartDashboard.putNumber("7 feet per second is", drivetrain.getLeft().getModel().toNativeUnitPosition(LengthKt.getFeet(7)).getValue());

		SmartDashboard.putNumber("Current Gyro angle", drivetrain.getGyro());

		// SmartDashboard.putData(drivetrain);
		// SmartDashboard.putNumber("Current elbow angle: ", SuperStructure.getInstance().getElbow().getMaster().getSensorPosition().getDegree());
		// SmartDashboard.putNumber("Current wrist angle: ", SuperStructure.getInstance().getWrist().getMaster().getSensorPosition().getDegree());
		// SmartDashboard.putData(superstructure);

		SmartDashboard.putData(autoState); //TODO test to see if it actually does the thing

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
