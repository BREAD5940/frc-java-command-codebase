package frc.robot.states;

import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.commands.auto.AutoMotion;
import frc.robot.commands.auto.AutoMotion.mHeldPiece;
import frc.robot.subsystems.superstructure.*;
import frc.robot.subsystems.superstructure.Superstructure;


/**
 * A state the robot superstructure (elevator, wrist, intake, etc.) can be in. Used mainly by the SuperstructurePlanner
 * 
 * @author Jocelyn McHugo
 */
public class SuperstructureState{

  private double elevatorHeight;
  private Wrist.WristPos wristAngle;
  private AutoMotion.mHeldPiece piece = mHeldPiece.NONE;

  /**
   * Create a new state with default params
   */
  public SuperstructureState(){
    this(RobotConfig.Superstructure.minElevatorHeight, Wrist.WristPos.CARGO, mHeldPiece.NONE);
  }

  /**
   * create a duplicate state from an existing state
   * @param existing
   *    the state you want to copy
   */
  public SuperstructureState(SuperstructureState existing){
    this.elevatorHeight=existing.getElevatorHeight();
    this.wristAngle=existing.getWristAngle();
    this.piece=existing.getHeldPiece();
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
  public SuperstructureState(double height, Wrist.WristPos angle, mHeldPiece piece){
    this.elevatorHeight = height;
    this.wristAngle = angle;
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
   * set the angle of the wrist for the state from the wrist presets in Wrist.java
   * @param angle
   *    the desired preset wrist position
   */
  public void setWristAngle(Wrist.WristPos angle){
    this.wristAngle = angle;
  }

  /**
   * set the status of the hatch intake for the state
   * @param open
   *    if the hatch intake is open
   */
  public void setHeldPiece(mHeldPiece piece){
    this.piece = piece;
  }

  /**
   * updates the state to the current positions of each part
   */
  public void updateToCurrent(){
    //TODO make seperate wrist angles, change intakeOpen from public boolean to get() function in Intake.java
    this.elevatorHeight = Superstructure.elevator.getHeight();
    this.wristAngle= Superstructure.wrist.presetAngle;
    // can't automatically set what piece is held w/o a bunch of sensors
  }


  public double getElevatorHeight(){
    return this.elevatorHeight;
  }

  public Wrist.WristPos getWristAngle(){
    return this.wristAngle;
  }

  public mHeldPiece getHeldPiece(){
    return this.piece;
  }
}