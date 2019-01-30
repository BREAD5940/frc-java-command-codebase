package frc.robot.states;

import frc.robot.RobotConfig;
import frc.robot.commands.auto.AutoMotion;
import frc.robot.commands.auto.AutoMotion.mHeldPiece;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Superstructure.iPosition;
import frc.robot.subsystems.superstructure.Wrist;

/**
 * A state the robot superstructure (elevator, wrist, intake, etc.) can be in. Used mainly by the SuperstructurePlanner
 * 
 * @author Jocelyn McHugo
 */
public class SuperstructureState{

  private double elevatorHeight;
  private IntakeAngle angle;
  private AutoMotion.mHeldPiece piece = mHeldPiece.NONE;

  /**
   * Create a new state with default params
   */
  public SuperstructureState(){
    this(RobotConfig.Superstructure.minElevatorHeight, iPosition.CARGO_GRAB, mHeldPiece.NONE);
  }

  /**
   * create a duplicate state from an existing state
   * @param existing
   *    the state you want to copy
   */
  public SuperstructureState(SuperstructureState existing){
    this(existing.elevatorHeight, existing.angle, existing.piece);
  }

  /**
   * create a new state with all params
   * @param height
   *    the height of the elevator for the state
   * @param angle
   *    the angle preset of the wrist for the state
   * @param piece
   *    the piece the robot is holding
   */
  public SuperstructureState(double height, IntakeAngle angle, mHeldPiece piece){
    this.elevatorHeight = height;
    this.angle = angle;
    this.piece = piece;
  }


  /**
   * set the height of the elevator for the state
   * @param height
   *    the height of the elevator
   */
  public void setElevatorHeight(double height){
    this.elevatorHeight=height;
  }

  /**
   * set the angle of the wrist for the state
   * @param angle
   *    the desired preset wrist position
   */
  public void setAngle(IntakeAngle angle){
    this.angle = angle;
  }


  /**
   * set the status of the hatch intake for the state
   * @param piece
   *    the piece the robot is currently holding
   */
  public void setHeldPiece(mHeldPiece piece){
    this.piece = piece;
  }

  /**
   * updates the state to the current positions of each part
   */
  public void updateToCurrent(){
    this.elevatorHeight = Superstructure.elevator.getHeight();
    this.angle= Superstructure.wrist.presetAngle;
    
    // can't automatically set what piece is held w/o a bunch of sensors
    // put them on the week 4 order sheet 
  }


  public double getElevatorHeight(){
    return this.elevatorHeight;
  }

  public IntakeAngle getAngle(){
    return this.angle;
  }

  public mHeldPiece getHeldPiece(){
    return this.piece;
  }
}