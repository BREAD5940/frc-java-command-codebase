package frc.robot.states;

import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.commands.auto.AutoMotion;
import frc.robot.commands.auto.AutoMotion.mHeldPiece;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.Wrist;

/**
 * A state the robot superstructure (elevator, wrist, intake, etc.) can be in. Used mainly by the SuperstructurePlanner
 * 
 * @author Jocelyn McHugo
 */
public class SuperstructureState{

  private double elevatorHeight;
  private Wrist.WristPos wristAngle;
  private boolean stanAngle = true;
  private double rawAngle;
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
    this.stanAngle=true;
  }

  /**
   * set the angle of the wrist for the state with a raw double value
   * @param angle
   *    the desired wrist angle
   */
  public void setWristAngle(double angle){
    this.rawAngle=angle;
    this.stanAngle=false;
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
    this.elevatorHeight = Robot.superstructure.elevator.getHeight();
    if(Robot.superstructure.wrist.stanAngle){
      this.wristAngle= Robot.superstructure.wrist.presetAngle;
    }else{
      this.rawAngle=Robot.superstructure.wrist.rawAngle;
    }
    
    // can't automatically set what piece is held w/o a bunch of sensors
  }


  public double getElevatorHeight(){
    return this.elevatorHeight;
  }

  public Wrist.WristPos getWristAngle(){
    return this.wristAngle;
  }

  public double getRawWristAngle(){
    return this.rawAngle;
  }

  public mHeldPiece getHeldPiece(){
    return this.piece;
  }

  public boolean getStanAngle(){
    return this.stanAngle;
  }
}