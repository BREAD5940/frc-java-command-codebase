package frc.robot.subsystems.superstructure;

import java.util.ArrayList;
import java.util.Arrays;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Mass;
import org.ghrobotics.lib.mathematics.units.MassKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.team254.lib.physics.DCMotorTransmission;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.commands.auto.AutoMotion;
import frc.robot.commands.auto.AutoMotion.HeldPiece;
import frc.robot.commands.subsystems.superstructure.SuperStructureTelop;
import frc.robot.lib.PIDSettings;
import frc.robot.lib.PIDSettings.FeedbackMode;
import frc.robot.lib.obj.InvertSettings;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.planners.SuperstructurePlanner;
import frc.robot.states.ElevatorState;
import frc.robot.states.IntakeAngle;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.DriveTrain;
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
	private static double currentDTVelocity; //in ft/sec
	public static Length currentSetHeight, lastSH = LengthKt.getInch(70), lastLastSH = LengthKt.getInch(70);
	public SuperStructureState mReqState = new SuperStructureState();
	public SuperStructureState lastState = new SuperStructureState(updateState());
	private CommandGroup mCurrentCommandGroup;
	private ArrayList<SuperStructureState> mReqPath;
	private int cPathIndex = 0;
	private boolean currentPathComplete = false;
	public static Elevator elevator;
	public static Intake intake = new Intake(34);
	private SuperstructurePlanner planner = new SuperstructurePlanner();
	// public SuperStructureState mPeriodicIO = new SuperStructureState();
	private RotatingJoint mWrist, mElbow;
	private DCMotorTransmission kElbowTransmission, kWristTransmission;
	public static final Mass kHatchMass = MassKt.getLb(2.4); // FIXME check mass
	public static final Mass kCargoMass = MassKt.getLb(1); // FIXME check mass
	public static final RoundRotation2d kWristMin = RoundRotation2d.getDegree(-45); // relative
	public static final RoundRotation2d kWristMax = RoundRotation2d.getDegree(90); // relative
	public static final RoundRotation2d kElbowMin = RoundRotation2d.getDegree(-180); // absolute
	public static final RoundRotation2d kElbowMax = RoundRotation2d.getDegree(15); // absolute

	public static synchronized SuperStructure getInstance() {
		if (instance_ == null) {
			instance_ = new SuperStructure();
		}
		return instance_;
	}

	public Length getLastReqElevatorHeight() {
		return lastSH;
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

		mWrist = new Wrist(new PIDSettings(0.5d, 0, 0, 0, FeedbackMode.ANGULAR), 33, FeedbackDevice.CTRE_MagEncoder_Relative, 8, kWristMin, kWristMax, true /* FIXME check inverting! */,
				Constants.kWristLength, Constants.kWristMass); // FIXME the ports are wrong and check inverting!

		mElbow = new RotatingJoint(new PIDSettings(0.5d, 0, 0, 0, FeedbackMode.ANGULAR), Arrays.asList(31, 32), FeedbackDevice.CTRE_MagEncoder_Relative, 9.33, kElbowMin, kElbowMax,
				false /* FIXME should this be inverted? */, Constants.kElbowLength, Constants.kElbowMass);

		elevator = new Elevator(21, 22, 23, 24, EncoderMode.CTRE_MagEncoder_Relative,
				new InvertSettings(true, InvertType.FollowMaster, InvertType.FollowMaster, InvertType.OpposeMaster));

		mCurrentState = new SuperStructureState();

	}

	private SuperStructureState mCurrentState;

	public SuperStructureState getCurrentState() {
		return mCurrentState;
	}

	public static class iPosition {
		public static final IntakeAngle CARGO_GRAB = new IntakeAngle(new RotatingArmState(Rotation2dKt.getDegree(0)), new RotatingArmState(Rotation2dKt.getDegree(0)));
		public static final IntakeAngle CARGO_DOWN = new IntakeAngle(new RotatingArmState(Rotation2dKt.getDegree(0)), new RotatingArmState(Rotation2dKt.getDegree(0)));
		public static final IntakeAngle CARGO_PLACE = new IntakeAngle(new RotatingArmState(Rotation2dKt.getDegree(0)), new RotatingArmState(Rotation2dKt.getDegree(0)));
		public static final IntakeAngle CARGO_REVERSE = new IntakeAngle(new RotatingArmState(Rotation2dKt.getDegree(0)), new RotatingArmState(Rotation2dKt.getDegree(0)));
		public static final IntakeAngle HATCH = new IntakeAngle(new RotatingArmState(Rotation2dKt.getDegree(0)), new RotatingArmState(Rotation2dKt.getDegree(0)));
		public static final IntakeAngle HATCH_REVERSE = new IntakeAngle(new RotatingArmState(Rotation2dKt.getDegree(0)), new RotatingArmState(Rotation2dKt.getDegree(90)));

		public static final ArrayList<IntakeAngle> presets = new ArrayList<IntakeAngle>(Arrays.asList(CARGO_GRAB, CARGO_DOWN,
				CARGO_PLACE, CARGO_REVERSE, HATCH, HATCH_REVERSE));
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
	public void moveSuperstructureCombo(ElevatorState elevator, RotatingArmState elbow,
			RotatingArmState wrist) {
		moveSuperstructureCombo(new SuperStructureState(elevator, elbow, wrist));
	}

	/**
	* Move the superstructure based on a height, intake angle and wrist angle
	* TODO how do we go from held game piece to target angle?
	*/
	public void moveSuperstructureCombo(ElevatorState elevator, IntakeAngle intakeState) {
		moveSuperstructureCombo(new SuperStructureState(elevator, intakeState));
	}

	/**
	 * move a combination of the sub-subsystems of the superstructure
	 * 
	 * @param mReqState_ the state that we want the superstructure to end up in
	 * 
	 * @returnW
	 *    the command group necessary to safely move the superstructure
	 */
	public void moveSuperstructureCombo(SuperStructureState mRequState_) {

		// TODO the wrist angle is mega broken because it's solely based on the currently held game piece 
		// this.mCurrentCommandGroup = planner.plan(mReqState, mCurrentState);
		this.mReqPath = planner.plan(mRequState_, mCurrentState);
		mReqState = mRequState_; // TODO I still don't trust mReqState
		// return this.mCurrentCommandGroup;
	}

	/**
	 * move only the elevator of the superstructure
	 * @param height
	 *    the height to raise the elevator to
	 * @return
	 *    the command group necessary to safely move the superstructure
	 */
	public void moveSuperstructureElevator(Length height) {
		// updateState();
		this.moveSuperstructureCombo(new ElevatorState(height), getCurrentState().getElbow(), getCurrentState().getWrist());
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
	public void moveSuperstructureAngle(IntakeAngle intakeState, AutoMotion.HeldPiece piece) {
		// updateState();
		this.moveSuperstructureCombo(mReqState.getElevator(), intakeState);
	}

	@Override
	protected void initDefaultCommand() {
		// pretty sure it doesn't need a default command, so leaving this empty
		// Actually yeah all that you really need is the buttons
		// well also jogging with joysticks but eehhhh
		// actually that should be the default command, hot prank
		setDefaultCommand(new SuperStructureTelop());
	}

	public SuperStructureState updateState() {
		// ElevatorState mCurrent = mCurrentState.elevator;
		getElevator().getCurrentState();
		getWrist().getCurrentState();
		getElbow().getCurrentState();

		this.mCurrentState = new SuperStructureState(
				elevator.getCurrentState(),
				// mWrist.getCurrentState(),
				// mElbow.getCurrentState());
				mWrist.getCurrentState(),
				mElbow.getCurrentState());
		return mCurrentState;
	}

	public RotatingJoint getWrist() {
		if (mWrist == null)
			mWrist = new Wrist(new PIDSettings(0.5d, 0, 0, 0, FeedbackMode.ANGULAR), 33, FeedbackDevice.CTRE_MagEncoder_Relative, 8, kWristMin, kWristMax, true /* FIXME check inverting! */,
					Constants.kWristLength, Constants.kWristMass); // FIXME the ports are wrong and check inverting!

		// 		mWrist = new Wrist(new PIDSettings(0.5d, 0, 0, 0, FeedbackMode.ANGULAR), 33, FeedbackDevice.CTRE_MagEncoder_Relative, 8, kWristMin, kWristMax, true /* FIXME check inverting! */,
		// 		Constants.kWristLength, Constants.kWristMass); // FIXME the ports are wrong and check inverting!

		// mElbow = new RotatingJoint(new PIDSettings(0.5d, 0, 0, 0, FeedbackMode.ANGULAR), Arrays.asList(31, 32), FeedbackDevice.CTRE_MagEncoder_Relative, 9.33, kElbowMin, kElbowMax,
		// 		false /* FIXME should this be inverted? */, Constants.kElbowLength, Constants.kElbowMass);

		return mWrist;
	}

	public RotatingJoint getElbow() {
		if (mElbow == null)
			mElbow = new RotatingJoint(new PIDSettings(0.5d, 0, 0, 0, FeedbackMode.ANGULAR), Arrays.asList(31, 32), FeedbackDevice.CTRE_MagEncoder_Relative, 9.33, kElbowMin, kElbowMax,
					false /* FIXME should this be inverted? */, Constants.kElbowLength, Constants.kElbowMass);
		return mElbow;
	}

	public DCMotorTransmission getWTransmission() {
		return kWristTransmission;
	}

	public DCMotorTransmission getETransmission() {
		return kElbowTransmission;
	}

	public Elevator getElevator() {
		if (elevator == null)
			elevator = new Elevator(21, 22, 23, 24, EncoderMode.CTRE_MagEncoder_Relative,
					new InvertSettings(false, InvertType.FollowMaster, InvertType.OpposeMaster, InvertType.OpposeMaster));
		return elevator;
	}

	public void move(SuperStructureState requState){
		//former superstructure periodic
		updateState();

		SuperStructureState prevState = lastState;
		// double mCurrentWristTorque = Math.abs(SuperStructure.getInstance().calculateWristTorque(prevState)); // torque due to gravity and elevator acceleration, newton meters
		// double mCurrentElbowTorque = Math.abs(SuperStructure.getInstance().calculateElbowTorques(prevState, mCurrentWristTorque)); // torque due to gravity and elevator acceleration, newton meters

		// double wristVoltageGravity = SuperStructure.getInstance().getWTransmission().getVoltageForTorque(SuperStructure.getInstance().updateState().getWrist().velocity.getValue(), mCurrentWristTorque);
		// double elbowVoltageGravity = SuperStructure.getInstance().getETransmission().getVoltageForTorque(SuperStructure.getInstance().updateState().getElbow().velocity.getValue(), mCurrentElbowTorque);
		double elevatorPercentVbusGravity = getElevator().getVoltage(updateState()) / 12;//getElevator().getMaster().getBusVoltage();		

		// if (Math.abs(mOI.getWristAxis()) > 0.07) {
		// SuperStructure.getInstance().getWrist().getMaster().set(ControlMode.Position, mRequState.getWrist().angle);

		// SuperStructure.getInstance().getElbow().getMaster().set(ControlMode.Position, mRequState.getElbow().angle);
		SuperStructureState stateSetpoint = plan(requState);

		getWrist().requestAngle(stateSetpoint.getWrist().angle); // div by 12 because it expects a throttle
		getElbow().requestAngle(stateSetpoint.getElbow().angle); // div by 12 because it expects a throttle
		getElevator().setPositionArbitraryFeedForward(stateSetpoint.getElevator().height, elevatorPercentVbusGravity / 12d);
		// getElevator().getMaster().set(ControlMode.PercentOutput, elevatorPercentVbusGravity);
	}

	public SuperStructureState plan(SuperStructureState mReqState) {

		mReqPath = planner.plan(mReqState, mCurrentState);

		Length reqSetHeight = mReqPath.get(0).getElevatorHeight();

		Length currentSetHeight = reqSetHeight;
		currentDTVelocity = Math.abs((DriveTrain.getInstance().getLeft().getFeetPerSecond() + DriveTrain.getInstance().getRight().getFeetPerSecond()) / 2);
		currentSetHeight = reqSetHeight;

		if (currentDTVelocity > 5) {
			currentSetHeight = LengthKt.getInch(0.310544 * Math.pow(currentDTVelocity, 2) - 11.7656 * currentDTVelocity + 119.868); //FIXME this is a regression based on arb. values. update after testing
			if (currentSetHeight.getInch() > reqSetHeight.getInch()) {
				currentSetHeight = reqSetHeight;
			}
		}

		currentSetHeight = (currentSetHeight.plus(lastSH.plus(lastLastSH))).div(3);

		lastLastSH = lastSH;
		lastSH = currentSetHeight;

		lastState = new SuperStructureState(new ElevatorState(currentSetHeight), mReqPath.get(0).getAngle());

		return new SuperStructureState(new ElevatorState(currentSetHeight), mReqPath.get(0).getAngle());
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
