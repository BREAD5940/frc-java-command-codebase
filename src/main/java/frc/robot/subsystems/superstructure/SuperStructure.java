package frc.robot.subsystems.superstructure;

import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.MassKt;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.auto.AutoMotion;
import frc.robot.lib.PIDSettings;
import frc.robot.lib.PIDSettings.FeedbackMode;
import frc.robot.lib.obj.InvertSettings;
import frc.robot.lib.SuperstructurePlanner;
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
  public static Intake intake = new Intake();
  private SuperstructurePlanner planner = new SuperstructurePlanner();
  public SuperStructureState mPeriodicIO = new SuperStructureState();
  private RotatingJoint mWrist, mElbow;
  
  public static synchronized SuperStructure getInstance() {
    if ( instance_ == null ) {
      instance_ = new SuperStructure();
    }
    return instance_;
  }

  public enum ElevatorPresets {
    LOW_ROCKET_PORT(27),
    MIDDLE_ROCKET_PORT(55),
    HIGH_ROCKET_PORT(84),
    LOW_ROCKET_HATCH(19),
    MIDDLE_ROCKET_HATCH(47),
    HIGH_ROCKET_HATCH(75),

    CARGO_SHIP_HATCH(20),
    // TODO this should be even with the low rocket hatch. According to the game manual, it isn't
    CARGO_SHIP_WALL(31);
    //top of wall

    private Length height;

    ElevatorPresets(int height_){
      this.height = LengthKt.getInch(height_);
    }
    public Length getValue(){
      return height;
    }
  }

  private SuperStructure(){
    super("SuperStructure");
    mWrist = new RotatingJoint(new PIDSettings(1d, 0, 0, 0, FeedbackMode.ANGULAR), 37, FeedbackDevice.CTRE_MagEncoder_Relative, 
        LengthKt.getInch(6), MassKt.getLb(15), 4.9474);
    mElbow = new RotatingJoint(new PIDSettings(1d, 0, 0, 0, FeedbackMode.ANGULAR), 40, FeedbackDevice.CTRE_MagEncoder_Relative, 
        LengthKt.getInch(6), MassKt.getLb(2), 9.8933);
  }


  private SuperStructureState mCurrentState = new SuperStructureState();

  public static class iPosition{
    public static final IntakeAngle CARGO_GRAB = new IntakeAngle(new RotatingArmState(Rotation2dKt.getDegree(0)), new RotatingArmState(Rotation2dKt.getDegree(0)));
    public static final IntakeAngle CARGO_DOWN = new IntakeAngle(new RotatingArmState(Rotation2dKt.getDegree(0)), new RotatingArmState(Rotation2dKt.getDegree(0)));
    public static final IntakeAngle CARGO_DROP = new IntakeAngle(new RotatingArmState(Rotation2dKt.getDegree(0)), new RotatingArmState(Rotation2dKt.getDegree(0)));
    public static final IntakeAngle CARGO_REVERSE = new IntakeAngle(new RotatingArmState(Rotation2dKt.getDegree(0)), new RotatingArmState(Rotation2dKt.getDegree(0)));
    public static final IntakeAngle HATCH = new IntakeAngle(new RotatingArmState(Rotation2dKt.getDegree(0)), new RotatingArmState(Rotation2dKt.getDegree(0)));
    public static final IntakeAngle HATCH_REVERSE = new IntakeAngle(new RotatingArmState(Rotation2dKt.getDegree(0)), new RotatingArmState(Rotation2dKt.getDegree(0)));

    public static final ArrayList<IntakeAngle> presets = new ArrayList<IntakeAngle>(Arrays.asList(CARGO_GRAB, CARGO_DOWN,
                            CARGO_DROP, CARGO_REVERSE, HATCH, HATCH_REVERSE));
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
  public CommandGroup moveSuperstructureCombo(SuperStructureState mRequState_){
    mCurrentState = updateState();

    // TODO the wrist angle is mega broken because it's solely based on the currently held game piece
    mReqState = mRequState_; 
    
    if(!(mReqState==mCurrentState)){
      this.mCurrentCommandGroup = planner.plan(mReqState, mCurrentState);
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
  public CommandGroup moveSuperstructureElevator(Length height){
    updateState();
    return this.moveSuperstructureCombo(new ElevatorState(height), mElbow.getCurrentState(), mWrist.getCurrentState());
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
  public CommandGroup moveSuperstructureAngle(IntakeAngle intakeState, AutoMotion.HeldPiece piece){
    updateState();
    return this.moveSuperstructureCombo( mCurrentState.elevator, intakeState );
  }

  @Override
  protected void initDefaultCommand() {
    // pretty sure it doesn't need a default command, so leaving this empty
    // Actually yeah all that you really need is the buttons
    // well also jogging with joysticks but eehhhh
    // actually that should be the default command, hot prank
  }

  /* so the problem right now is that everything is abstracted
  which is a great problem to have, except for the part where everything
  connects together. We need a way to calculate feedforward voltages and velocities based on
  current angles and desired angles respectively and feed it to the talons. I'm trying to
  think of a good way to do this, and I think the best way to do it might be to put it on a notifier?? 
  that would put everyhing on another thread. Otherwise maybe make a looper abstract class
  and have those calculations happen every tick. Either way the motor states would need to be
  calculated *as fast as possible*. I don't think that a command is the best way to do this either, 
  because we need the sequential/paralell nature of command groups for safe superstructure movement
  while preserving the "this happens every tick" part of the whole system.
  Yeah, heck it. I'm making a looper abstract class which puts everything on a notifier so we 
  preserve abstraction. That means that this needs an init() and execute() function, as well as an end() function.
  Furthermore I think we should maybe get rid of the wrist subsystem and elevator subsystem - all their
  suff is relaced with FalconSRX<Length> or FalconSRX<Rotation2d>
  
  TODO we also need to do the encoders and stuff
  */

  public SuperStructureState updateState() {
    this.mCurrentState = new SuperStructureState(
      elevator.getCurrentState(mCurrentState.elevator),
      mWrist.getCurrentState(),
      mElbow.getCurrentState()
    );
    return mCurrentState;
  }
  
  public RotatingJoint getWrist() {
    return mWrist;
  }

  public RotatingJoint getElbow() {
    return mElbow;
  }

  @Override
  public void periodic() {
    System.out.println("hullo there");
  }

  // TODO check this math coz you might be able to make it more efficient
  public double calculateWristTorque(SuperStructureState state) {
    double x1= (getWrist().kArmLength.getValue()) * state.jointAngles.getWrist().angle.getCos();
    double torqueGravity = (getWrist().kArmMass.getValue() * 9.8 * x1);
    double torqueAccel = getWrist().kArmMass.getValue() * state.elevator.acceleration.getValue() * x1;
    double totalTorque = torqueGravity + torqueAccel;
    return totalTorque;
  }

  // TODO check this math coz you might be able to make it more efficient
  public double calculateElbowTorques(SuperStructureState state, double wristTorque) {
    double x2= (getElbow().kArmLength.getValue()) * state.jointAngles.getWrist().angle.getCos();
    double torqueGravity = (getElbow().kArmMass.getValue() * 9.8 * x2); 
    double torqueAccel = getElbow().kArmMass.getValue() * state.elevator.acceleration.getValue() * x2;
    double Big = (getElbow().kArmMass.getValue() * getElbow().kArmLength.getValue() * 9.8) + wristTorque;
    return torqueGravity + torqueAccel + Big;
  }
}