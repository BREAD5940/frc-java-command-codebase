package frc.robot.subsystems.superstructure;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.auto.AutoMotion;
import frc.robot.planners.SuperstructurePlanner;
import frc.robot.lib.SuperstructurePlanner;
import frc.robot.states.IntakeAngle;
import frc.robot.states.SuperstructureState;
import frc.robot.subsystems.Intake;

/**
 * First level of control for the superstructure of the robot. Contains all the
 * callable methods for the superstructure.
 * 
 * @author Jocelyn McHugo
 */
public class Superstructure extends Subsystem {

  private SuperstructureState mReqState = new SuperstructureState();
  private CommandGroup mCurrentCommandGroup;
  public static Elevator elevator = new Elevator();
  public static Wrist wrist = new Wrist();
  public static Intake intake = new Intake();
  private SuperstructurePlanner planner = new SuperstructurePlanner();
  
  public Superstructure(){}


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
  }

}