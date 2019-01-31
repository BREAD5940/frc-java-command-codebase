package frc.robot.subsystems.superstructure;

import java.util.ArrayList;
import java.util.Arrays;

import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotConfig.wrist;
import frc.robot.commands.auto.AutoMotion;
import frc.robot.lib.Logger;
import frc.robot.lib.LoopingSubsystem;
import frc.robot.lib.SuperstructurePlanner;
import frc.robot.lib.AbstractRotatingArm.RotatingArmPeriodicIO;
import frc.robot.states.IntakeAngle;
import frc.robot.states.SuperstructureState;
import frc.robot.subsystems.Intake;

/**
 * First level of control for the superstructure of the robot. Contains all the
 * callable methods for the superstructure.
 * 
 * @author Jocelyn McHugo
 */
public class Superstructure extends LoopingSubsystem {

  private Superstructure instance_;
  private SuperstructureState mReqState = new SuperstructureState();
  private CommandGroup mCurrentCommandGroup;
  public static Elevator elevator = new Elevator();
  public static Wrist wrist = new Wrist();
  public static Intake intake = new Intake();
  public static Elbow elbow = new Elbow();
  private SuperstructurePlanner planner = new SuperstructurePlanner();
  public SuperStructurePeriodicIO mPeriodicIO = new SuperStructurePeriodicIO();
  
  public synchronized Superstructure getInstance() {
    if ( instance_ == null ) {
      instance_ = new Superstructure();
    }
    return instance_;
  }

  public Superstructure(){
    super(1);
    startLooper();
  }


  private SuperstructureState mCurrentState = new SuperstructureState();

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
   * @param height
   *    the height to raise the elevator to
   * @param angle
   *    the preset angle to set the wrist to
   * @param piece
   *    the piece the robot is currently holding -- necessary for wrist movements
   * @return
   *    the command group necessary to safely move the superstructure
   */
  public CommandGroup moveSuperstructureCombo(double height, IntakeAngle angle, AutoMotion.mHeldPiece piece){
    mCurrentState.updateToCurrent();
    mReqState.setElevatorHeight(height);
    mReqState.setAngle(angle);
    mReqState.setHeldPiece(piece);

    mCurrentState.updateToCurrent();   
    
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
    mCurrentState.updateToCurrent();  
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

  public void initilize() {
    if(mPeriodicIO == null) mPeriodicIO = new SuperStructurePeriodicIO();
    if(mPeriodicIO.elbow == null) mPeriodicIO.elbow = new RotatingArmPeriodicIO();
    if(mPeriodicIO.wrist == null) mPeriodicIO.wrist = new RotatingArmPeriodicIO();
    // TODO move the superstructure to a known good starting position
    // This should be called on auto init, coz that's when the motors init
    System.out.println("Superstructure looper init-ed");
  }

  int i=0;

  public void execute() {
    i++;
    // TODO calculate feedforward voltages and shit
    // if(mPeriodicIO == null) mPeriodicIO = new SuperStructurePeriodicIO();
    // if(mPeriodicIO.elbow == null) mPeriodicIO.elbow = new RotatingArmPeriodicIO();
    // if(mPeriodicIO.wrist == null) mPeriodicIO.wrist = new RotatingArmPeriodicIO();

    elbow.setSetpoint(Rotation2dKt.getDegree(i));
    wrist.setSetpoint(Rotation2dKt.getDegree(20));



    System.out.println(elbow.mPeriodicIO.toString() + ", " + wrist.mPeriodicIO.toString());
  }

  public void end() {}

  
  public class SuperStructurePeriodicIO {
    public RotatingArmPeriodicIO elbow, wrist;
    // public RotatingArmPeriodicIO wrist = new RotatingArmPeriodicIO();
    public SuperStructurePeriodicIO() {
      elbow = new RotatingArmPeriodicIO();
      wrist = new RotatingArmPeriodicIO();
    }
    @Override
    public String toString() {
      return elbow.toString() + ", " + wrist.toString();
    }
  }

}