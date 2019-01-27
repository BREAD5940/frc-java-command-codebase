package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.auto.AutoMotion;
import frc.robot.lib.SuperstructurePlanner;
import frc.robot.states.SuperstructureState;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.superstructure.Wrist.WristPos;

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
  public CommandGroup moveSuperstructureCombo(double height, WristPos angle, AutoMotion.mHeldPiece piece){
    mCurrentState.updateToCurrent();
    mReqState.setElevatorHeight(height);
    mReqState.setWristAngle(angle);
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
    return this.moveSuperstructureCombo(height, mCurrentState.getWristAngle(), mCurrentState.getHeldPiece());
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
  public CommandGroup moveSuperstructureWrist(Wrist.WristPos angle, AutoMotion.mHeldPiece piece){
    mCurrentState.updateToCurrent();  
    return this.moveSuperstructureCombo(mCurrentState.getElevatorHeight(), angle, piece);
  }


  /**
   * sets only the wrist of the superstructure
   * @param angle
   *    the RAW angle to set the wrist to
   * @param piece
   *    the piece the robot is currently holding -- necessary for wrist movements
   * @return
   *    the command group necessary to safely move the superstructure
   */
  public CommandGroup moveSuperstructureWrist(double angle, AutoMotion.mHeldPiece piece){
    mCurrentState.updateToCurrent();
    mReqState.setElevatorHeight(mCurrentState.getElevatorHeight());
    mReqState.setWristAngle(angle);
    mReqState.setHeldPiece(piece); 
    
    if(!(mReqState==mCurrentState)){
      this.mCurrentCommandGroup = planner.plan(mReqState, mCurrentState);
    }

    return this.mCurrentCommandGroup;
  }


  @Override
  protected void initDefaultCommand() {
    // pretty sure it doesn't need a default command, so leaving this empty
  }

}