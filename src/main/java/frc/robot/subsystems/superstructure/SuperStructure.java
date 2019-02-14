package frc.robot.subsystems.superstructure;

import java.util.ArrayList;
import java.util.Arrays;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Mass;
import org.ghrobotics.lib.mathematics.units.MassKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.team254.lib.physics.DCMotorTransmission;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.commands.auto.AutoMotion;
import frc.robot.commands.auto.AutoMotion.HeldPiece;
import frc.robot.commands.subsystems.superstructure.SuperStructureTelop;
import frc.robot.commands.subsystems.superstructure.SuperstructureGoToState;
import frc.robot.lib.obj.InvertSettings;
import frc.robot.planners.SuperstructurePlanner;
import frc.robot.states.ElevatorState;
import frc.robot.states.IntakeAngle;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.superstructure.Elevator.EncoderMode;
import frc.robot.subsystems.superstructure.RotatingJoint.RotatingArmState;

/**
 * First level of control for the superstructure of the robot. Contains all the
 * callable methods for the superstructure.
 * 
 * @author Jocelyn McHugo
 */
public class SuperStructure extends Subsystem {

	private static SuperStructure instance_;
	private SuperStructureState mReqState = new SuperStructureState();
	private CommandGroup mCurrentCommandGroup;
	public static Elevator elevator = new Elevator(21, 22, 23, 24, EncoderMode.CTRE_MagEncoder_Relative,
			new InvertSettings(false, InvertType.FollowMaster, InvertType.OpposeMaster, InvertType.OpposeMaster));
	public static Intake intake;// = new Intake();
	private SuperstructurePlanner planner = new SuperstructurePlanner();
	// public SuperStructureState mPeriodicIO = new SuperStructureState();
	private RotatingJoint mWrist, mElbow;
	private DCMotorTransmission kElbowTransmission, kWristTransmission;
	public static final Mass kHatchMass = MassKt.getLb(2.4); // FIXME check mass
	public static final Mass kCargoMass = MassKt.getLb(1); // FIXME check mass

	public static synchronized SuperStructure getInstance() {
		if (instance_ == null) {
			instance_ = new SuperStructure();
		}
		return instance_;
	}

	public enum ElevatorPresets {
		LOW_ROCKET_PORT(27), MIDDLE_ROCKET_PORT(55), HIGH_ROCKET_PORT(84), LOW_ROCKET_HATCH(19), MIDDLE_ROCKET_HATCH(47), HIGH_ROCKET_HATCH(75),

		CARGO_SHIP_HATCH(20),
		// TODO this should be even with the low rocket hatch. According to the game manual, it isn't
		CARGO_SHIP_WALL(31);
		//top of wall

		private Length height;

		ElevatorPresets(int height_) {
			this.height = LengthKt.getInch(height_);
		}

		public Length getValue() {
			return height;
		}
	}

	private SuperStructure() {
		super("SuperStructure");
		kElbowTransmission = new DCMotorTransmission(Constants.kElbowSpeedPerVolt, Constants.kElbowTorquePerVolt, Constants.kElbowStaticFrictionVoltage);

		kWristTransmission = new DCMotorTransmission(Constants.kWristSpeedPerVolt, Constants.kWristTorquePerVolt, Constants.kWristStaticFrictionVoltage);

		// mWrist = new RotatingJoint(new PIDSettings(1d, 0, 0, 0, FeedbackMode.ANGULAR), 37, FeedbackDevice.CTRE_MagEncoder_Relative, false /* FIXME check inverting! */,
		// 		Constants.kWristLength, Constants.kWristMass); // FIXME the ports are wrong and check inverting!

		// mElbow = new RotatingJoint(new PIDSettings(1d, 0, 0, 0, FeedbackMode.ANGULAR), Arrays.asList(38, 39), FeedbackDevice.CTRE_MagEncoder_Relative,
		// 		false /* FIXME should this be inverted? */, Constants.kElbowLength, Constants.kElbowMass);
	}

	private SuperStructureState mCurrentState = new SuperStructureState();

	public static class iPosition {
		public static final IntakeAngle CARGO_GRAB = new IntakeAngle(new RotatingArmState(Rotation2dKt.getDegree(0)), new RotatingArmState(Rotation2dKt.getDegree(0)));
		public static final IntakeAngle CARGO_DOWN = new IntakeAngle(new RotatingArmState(Rotation2dKt.getDegree(0)), new RotatingArmState(Rotation2dKt.getDegree(0)));
		public static final IntakeAngle CARGO_DROP = new IntakeAngle(new RotatingArmState(Rotation2dKt.getDegree(0)), new RotatingArmState(Rotation2dKt.getDegree(0)));
		public static final IntakeAngle CARGO_REVERSE = new IntakeAngle(new RotatingArmState(Rotation2dKt.getDegree(0)), new RotatingArmState(Rotation2dKt.getDegree(0)));
		public static final IntakeAngle HATCH = new IntakeAngle(new RotatingArmState(Rotation2dKt.getDegree(0)), new RotatingArmState(Rotation2dKt.getDegree(0)));
		public static final IntakeAngle HATCH_REVERSE = new IntakeAngle(new RotatingArmState(Rotation2dKt.getDegree(0)), new RotatingArmState(Rotation2dKt.getDegree(0)));

		public static final ArrayList<IntakeAngle> presets = new ArrayList<IntakeAngle>(Arrays.asList(CARGO_GRAB, CARGO_DOWN,
				CARGO_DROP, CARGO_REVERSE, HATCH, HATCH_REVERSE));
	}

	public boolean setReqState(SuperStructureState reqState) {
		if (!(planner.checkValidState(reqState)))
			return false;
		this.mReqState = reqState;
		return true;
	}

	/**
	 * Move the superstructure based on a height, intake angle and wrist angle
	 * TODO how do we go from held game piece to target angle?
	 */
	public CommandGroup moveSuperstructureCombo(ElevatorState elevator, RotatingArmState elbow,
			RotatingArmState wrist) {
		return moveSuperstructureCombo(new SuperStructureState(elevator, elbow, wrist));
	}

	/**
	* Move the superstructure based on a height, intake angle and wrist angle
	* TODO how do we go from held game piece to target angle?
	*/
	public CommandGroup moveSuperstructureCombo(ElevatorState elevator, IntakeAngle intakeState) {
		return moveSuperstructureCombo(new SuperStructureState(elevator, intakeState));
	}

	/**
	 * move a combination of the sub-subsystems of the superstructure
	 * 
	 * @param mReqState_ the state that we want the superstructure to end up in
	 * 
	 * @returnW
	 *    the command group necessary to safely move the superstructure
	 */
	public CommandGroup moveSuperstructureCombo(SuperStructureState mRequState_) {

		// TODO the wrist angle is mega broken because it's solely based on the currently held game piece 
		// this.mCurrentCommandGroup = planner.plan(mReqState, mCurrentState);
		ArrayList<SuperStructureState> path = planner.plan(mRequState_, mCurrentState);
		this.mCurrentCommandGroup = new CommandGroup("Superstructure Path");
		for (int i = 0; i < path.size() - 1; i++) {
			mCurrentCommandGroup.addSequential(new SuperstructureGoToState(path.get(i)));
		}

		return this.mCurrentCommandGroup;
	}

	/**
	 * move only the elevator of the superstructure
	 * @param height
	 *    the height to raise the elevator to
	 * @return
	 *    the command group necessary to safely move the superstructure
	 */
	public CommandGroup moveSuperstructureElevator(Length height) {
		updateState();
		return this.moveSuperstructureCombo(new ElevatorState(height), mReqState.getElbow(), mReqState.getWrist());
	}

	/**
	 * move only the wrist of the superstructure
	 * @param angle
	 *    the preset angle to set the wrist to
	 * @param piece
	 *    the piece the robot is currently holding -- necessary for wrist movements
	 * @return
	 *    the command group necessary to safely move the superstructure
	 */
	public CommandGroup moveSuperstructureAngle(IntakeAngle intakeState, AutoMotion.HeldPiece piece) {
		updateState();
		return this.moveSuperstructureCombo(mReqState.getElevator(), intakeState);
	}

	@Override
	protected void initDefaultCommand() {
		// pretty sure it doesn't need a default command, so leaving this empty
		// Actually yeah all that you really need is the buttons
		// well also jogging with joysticks but eehhhh
		// actually that should be the default command, hot prank
		setDefaultCommand(new SuperStructureTelop(getInstance()));
	}

	public SuperStructureState updateState() {
		this.mCurrentState = new SuperStructureState(
				elevator.getCurrentState(mCurrentState.elevator),
				// mWrist.getCurrentState(),
				// mElbow.getCurrentState());
				new RotatingArmState(),
				new RotatingArmState());
		return mCurrentState;
	}

	public RotatingJoint getWrist() {
		return mWrist;
	}

	public RotatingJoint getElbow() {
		return mElbow;
	}

	public Elevator getElevator() {
		if(elevator == null) elevator = new Elevator(21, 22, 23, 24, EncoderMode.CTRE_MagEncoder_Relative,
			new InvertSettings(false, InvertType.FollowMaster, InvertType.OpposeMaster, InvertType.OpposeMaster));
		return elevator;
	}

	@Override
	public void periodic() {
		// this calculates gravity feed forwards based off of the current state and requested state
		// make sure to keep these up to date!!!
		updateState();

		// Make for sure for real real that the voltage is always positive
		// double mCurrentWristTorque = Math.abs(calculateWristTorque(this.mCurrentState)); // torque due to gravity and elevator acceleration, newton meters
		// double mCurrentElbowTorque = Math.abs(calculateElbowTorques(this.mCurrentState, mCurrentWristTorque)); // torque due to gravity and elevator acceleration, newton meters

		// double wristVoltageGravity = kWristTransmission.getVoltageForTorque(this.mCurrentState.getWrist().velocity.getValue(), mCurrentWristTorque);
		// double elbowVoltageGravity = kElbowTransmission.getVoltageForTorque(this.mCurrentState.getElbow().velocity.getValue(), mCurrentElbowTorque);
		double elevatorVoltageGravity = 0;// = elevator.getVoltage(this.mCurrentState);

		// TODO velocity planning? or just let talon PID figure itself out
		// How about maybe motion magic?

		// TODO is mReqState up to date?
		// getWrist().setPositionArbitraryFeedForward(mReqState.getWrist().angle /* the wrist angle setpoint */, wristVoltageGravity / 12d); // div by 12 because it expects a throttle
		// getElbow().setPositionArbitraryFeedForward(mReqState.getElbow().angle /* the elbow angle setpoint */, elbowVoltageGravity / 12d); // div by 12 because it expects a throttle
		// getElevator().setPositionArbitraryFeedForward(mReqState.getElevator().height, elevatorVoltageGravity / 12d);

		SmartDashboard.putNumber("elevator height in inches", getElevator().getHeight().getInch());
	}

	/**
	 * Calculate the torque on the wrist due to gravity for a given state
	 * @param state the state of the superstructure 
	 * @return torque in newton meters on the wrist
	 */
	public double calculateWristTorque(SuperStructureState state) {
		Mass totalMass = getWrist().kArmMass;
		if (state.getHeldPiece() == HeldPiece.HATCH)
			totalMass.plus(kHatchMass);
		if (state.getHeldPiece() == HeldPiece.CARGO)
			totalMass.plus(kCargoMass);
		/* The distance from the pivot of the wrist to the center of mass */
		double x1 = (getWrist().kArmLength.getValue()) * Math.abs(state.jointAngles.getWrist().angle.getCos()); // absolute value so cosine is always positive
		/* The torque due to gravity  */
		double torqueGravity = (totalMass.getValue() * 9.8 * x1);
		/* The torque on the wrist due to acceleration of the elevator (assuming a rigid elbow) */
		double torqueAccel = totalMass.getValue() * state.elevator.acceleration.getValue() * x1;

		return torqueGravity + torqueAccel;
	}

	/**
	 * Calculate the elbow on the wrist due to gravity for a given state and wrist torque
	 * @param state the state of the superstructure 
	 * @param wristTorque the torque on the wrist
	 * @return torque in newton meters on the wrist
	 */
	public double calculateElbowTorques(SuperStructureState state, double wristTorque) {
		Mass wristTotalMass = getWrist().kArmMass;
		if (state.getHeldPiece() == HeldPiece.HATCH)
			wristTotalMass.plus(kHatchMass);
		if (state.getHeldPiece() == HeldPiece.CARGO)
			wristTotalMass.plus(kCargoMass);

		/* The distance from the pivot to the center of mass of the elbow */
		double x_2 = (getElbow().kArmLength.getMeter()) * Math.abs(state.jointAngles.getWrist().angle.getCos());// absolute value so cosine is always positive
		/* The torque due to gravity  */
		double torqueGravity = (getElbow().kArmMass.getKilogram() * 9.8 * x_2); // m_2 * g * x_2 
		/* The torque doe to acceleration on the wrist */
		double torqueAccel = getElbow().kArmMass.getKilogram() * state.elevator.acceleration.getValue() * x_2;
		// m_2 * g * x_2
		double torqueWrstComponent = (wristTotalMass.getKilogram() * getElbow().kArmLength.getMeter() * 2 /* double the distance from the joint to COM*/ * 9.8);

		return torqueGravity + torqueAccel + torqueWrstComponent + wristTorque;
	}
}
