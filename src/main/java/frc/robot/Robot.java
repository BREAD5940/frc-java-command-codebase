package frc.robot;

import frc.robot.commands.auto.miscActions.TerribleAutoChooser;
import org.ghrobotics.lib.debug.LiveDashboard;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2d;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.cscore.HttpCamera;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.ghrobotics.vision.*;
import frc.robot.commands.auto.Autonomous;
import frc.robot.commands.auto.miscActions.AutoMotion;
import frc.robot.commands.auto.routines.offseasonRoutines.TestRoutine;
import frc.robot.commands.subsystems.superstructure.PassThrough;
import frc.robot.commands.subsystems.superstructure.PassThrough.SyncedMove;
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
//import frc.ghrobotics.vision.FishyJeVois;

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
	public static DriveTrain drivetrain = DriveTrain.getInstance();
	public static SuperStructure superstructure = SuperStructure.getInstance();
	public static LimeLight limelight = LimeLight.getInstance();
	public static Autonomous auto = Autonomous.INSTANCE;
	public static DoubleSolenoid shifterDoubleSolenoid;
	public static DoubleSolenoid intakeDoubleSolenoid;
	public static DoubleSolenoid elevatorShifterDoubleSolenoid;
	public static AutoMotion m_auto;
	public static Compressor compressor = new Compressor(9);

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

		LiveWindow.disableAllTelemetry();

		SmartDashboard.putData(new TestRoutine());

		var jevois = JeVoisManager.INSTANCE;
		var tracker = TargetTracker.INSTANCE;
		var jevoisVision = VisionProcessing.INSTANCE;
		var limelightmanager = LimeLightManager.INSTANCE;

		Logger.clearLog();

		SmartDashboard.putData(Scheduler.getInstance()); //it'll let you see all the active commands and (I think) cancel them too

		SmartDashboard.putData(zeroElevatorWhileDisabled);

		//		mAutoChooser = new TerribleAutoChooser();
		//		mAutoChooser.addOptions();
		System.out.println("Auto chooser sent!");
		//		Trajectories.generateAllTrajectories();

		//		Network.INSTANCE.getAutoTab().getLayout("Path selection", BuiltInLayouts.kList).add(mAutoChooser.getChooser()).withSize(2, 5).withPosition(0, 0);
		Network.INSTANCE.getSuperStructureTab().add(superstructure);

		var camera = new HttpCamera("limelight", "http://10.59.40.11:5800/");

		Network.INSTANCE.getVisionTab().add(camera)
				.withPosition(0, 0)
				.withSize(6, 4);

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
		cmd.start();

		switch (RobotConfig.auto.auto_gear) {

		case HIGH:
			drivetrain.setHighGear();
			break;
		default:
			drivetrain.setLowGear();
		}

		drivetrain.zeroEncoders();
		System.out.println("Robot init'ed and encoders zeroed!");

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

	}

	private static Command zeroElevatorWhileDisabled = new ZeroElevatorDisabled();

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

		Autonomous.INSTANCE.JUSTS3NDIT();

	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {}

	@Override
	public void teleopInit() {

		SuperStructure.elevator.onDisable();

		// this line or comment it out.
		if (m_auto != null) {
			m_auto.getBigCommandGroup().cancel();
		}

		Autonomous.INSTANCE.JUSTSTOP();

		drivetrain.zeroGyro();
	}

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

		final boolean postTicks = false;

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


		SmartDashboard.putNumber("Robot X (feet) ", drivetrain.getLocalization().getRobotPosition().getTranslation().getX().getFeet());
		SmartDashboard.putNumber("Robot Y (feet) ", drivetrain.getLocalization().getRobotPosition().getTranslation().getY().getFeet());

		LiveDashboard.INSTANCE.setRobotX(drivetrain.getLocalization().getRobotPosition().getTranslation().getX().getFeet());
		LiveDashboard.INSTANCE.setRobotY(drivetrain.getLocalization().getRobotPosition().getTranslation().getY().getFeet());
		LiveDashboard.INSTANCE.setRobotHeading(drivetrain.getLocalization().getRobotPosition().getRotation().getRadian());

		SmartDashboard.putNumber("Current Gyro angle", drivetrain.getGyro());

		SmartDashboard.putData(autoState); //TODO test to see if it actually does the thing

	}

}
