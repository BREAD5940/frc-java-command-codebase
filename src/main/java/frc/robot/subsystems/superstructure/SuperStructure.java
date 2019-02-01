package frc.robot.subsystems.superstructure;

import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.MassKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.math.Rotation2d;
import frc.robot.commands.auto.AutoMotion;
import frc.robot.commands.auto.AutoMotion.mHeldPiece;
import frc.robot.lib.PIDSettings;
import frc.robot.lib.SuperstructurePlanner;
import frc.robot.lib.PIDSettings.FeedbackMode;
import frc.robot.states.ElevatorState;
import frc.robot.states.IntakeAngle;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.Intake;
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
  public static Elevator elevator = new Elevator();
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

  private SuperStructure(){
    super("SuperStructure");
    mWrist = new RotatingJoint(new PIDSettings(1d, 0, 0, 0, FeedbackMode.ANGULAR), 37, FeedbackDevice.CTRE_MagEncoder_Relative, 
        LengthKt.getInch(6), MassKt.getLb(15), 4.9474);
    mElbow = new RotatingJoint(new PIDSettings(1d, 0, 0, 0, FeedbackMode.ANGULAR), 40, FeedbackDevice.CTRE_MagEncoder_Relative, 
        LengthKt.getInch(6), MassKt.getLb(2), 9.8933);
  }


  private SuperStructureState mCurrentState = new SuperStructureState();

  public static class iPosition{
    public static final IntakeAngle CARGO_GRAB = new IntakeAngle(0,0);
    public static final IntakeAngle CARGO_DOWN = new IntakeAngle(0,0);
    public static final IntakeAngle CARGO_DROP = new IntakeAngle(0,0);
    public static final IntakeAngle CARGO_REVERSE = new IntakeAngle(0,0);
    public static final IntakeAngle HATCH = new IntakeAngle(0,0);
    public static final IntakeAngle HATCH_REVERSE = new IntakeAngle(0,0);

    public static final ArrayList<IntakeAngle> presets = new ArrayList<IntakeAngle>(Arrays.asList(CARGO_GRAB, CARGO_DOWN,
                            CARGO_DROP, CARGO_REVERSE, HATCH, HATCH_REVERSE));
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

    // TODO the wrist angle is mega broken
    mReqState = SuperStructureState.fromOther(mRequState_); // FIXME by making me an instance of superstructure state based on called params, the wrist is bork
    
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
  public CommandGroup moveSuperstructureElevator(double height){
    updateState();
    return this.moveSuperstructureCombo(height, mCurrentState.getAngle(), mCurrentState.getHeldPiece());
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
  public CommandGroup moveSuperstructureAngle(IntakeAngle angle, AutoMotion.mHeldPiece piece){
    mCurrentState.updateToCurrent();  
    return this.moveSuperstructureCombo(mCurrentState.getElevatorHeight(), angle, piece);
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
    mCurrentState = new SuperStructureState(
      mWrist.getCurrentState(),
      mElbow.getCurrentState(),
      elevator.getCurrentState()
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
  
  public double calculateWristTorque(SuperStructureState state) {
    double x1= (getWrist().kArmLength.getValue()) * state.wrist.angle.getCos();
    double torqueGravity = (getWrist().kArmMass.getValue() * 9.8 * x1);
    double torqueAccel = getWrist().kArmMass.getValue() * state.elevator.acceleration.getValue() * x1;
    double totalTorque = torqueGravity + torqueAccel;
    return totalTorque;
  }

  public double calculateElbowTorques(SuperStructureState state, double wristTorque) {
    double x2= (getElbow().kArmLength.getValue()) * state.wrist.angle.getCos();
    double torqueGravity = (getElbow().kArmMass.getValue() * 9.8 * x2);
    double torqueAccel = getElbow().kArmMass.getValue() * state.elevator.acceleration.getValue() * x2;
    double Big = (getElbow().kArmMass.getValue() * getElbow().kArmLength.getValue() * 9.8) + wristTorque;
    return torqueGravity + torqueAccel + Big;
  }
}