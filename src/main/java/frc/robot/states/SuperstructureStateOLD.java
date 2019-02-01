package frc.robot.states;

import frc.robot.RobotConfig;
import frc.robot.commands.auto.AutoMotion;
import frc.robot.commands.auto.AutoMotion.mHeldPiece;
import frc.robot.subsystems.superstructure.SuperStructure;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

/**
 * A state the robot superstructure (elevator, wrist, intake, etc.) can be in. Used mainly by the SuperstructurePlanner
 * (deprecited by superstructurestate in the superstructure class)
 * @deprecated
 * @author Jocelyn McHugo
 */
public class SuperstructureStateOLD{

  private double elevatorHeight;
  private IntakeAngle angle;
  private AutoMotion.mHeldPiece piece = mHeldPiece.NONE;

  /**
   * Create a new state with default params
   * @deprecated
   */
  public SuperstructureStateOLD(){
    this(RobotConfig.Superstructure.minElevatorHeight, iPosition.CARGO_GRAB, mHeldPiece.NONE);
  }

  /**
   * create a duplicate state from an existing state
   * @param existing
   *    the state you want to copy
   */
  public SuperstructureStateOLD(SuperstructureStateOLD existing){
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
  public SuperstructureStateOLD(double height, IntakeAngle angle, mHeldPiece piece){
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
    this.elevatorHeight = SuperStructure.elevator.getHeight();
    // this.angle = new IntakeAngle( Math.toDegrees(SuperStructure.getInstance().getElbow().getPosition().getValue()), 
    //         Math.toDegrees(SuperStructure.getInstance().getWrist().getPosition().getValue()));
    
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