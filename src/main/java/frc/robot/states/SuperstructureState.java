package frc.robot.states;

import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.subsystems.superstructure.*;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristPos;
import frc.robot.subsystems.superstructure.Superstructure;


/**
 * A state the robot superstructure (elevator, wrist, intake, etc.) can be in. Used mainly by the SuperStructurePlanner
 * 
 * @author Jocelyn McHugo
 */
public class SuperstructureState{

  private double elevatorHeight;
  private double wrist1Angle; //TODO maybe rename these to be more descriptive?
  private double wrist2Angle;
  private boolean hIntakeOpen = false; // TODO again with the descriptive thing

  /**
   * Create a new state with default params
   */
  public SuperstructureState(){
    this(RobotConfig.Superstructure.minElevatorHeight, RobotConfig.Superstructure.minWrist1Angle,
          RobotConfig.Superstructure.minWrist2Angle, false);
  }

  /**
   * create a duplicate state from an existing state
   * @param existing
   *    the state you want to copy
   */
  public SuperstructureState(SuperstructureState existing){
    this.elevatorHeight=existing.getElevatorHeight();
    this.wrist1Angle=existing.getWrist1Angle();
    this.wrist2Angle = existing.getWrist2Angle();
    this.hIntakeOpen=existing.getHIntakeOpen();
  }

  /**
   * create a new state with all params
   * @param height
   *    the height of the elevator for the state
   * @param angle1
   *    the first angle of the wrist for the state
   * @param angle2
   *    the second angle of the wrist for the state
   * @param hIntakeOpen
   *    if the hatch intake is open for the state
   */
  public SuperstructureState(double height, double angle1, double angle2, boolean hIntakeOpen){
    this.elevatorHeight = height;
    this.wrist1Angle = angle1;
    this.wrist2Angle = angle2;
    this.hIntakeOpen=hIntakeOpen;
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
    this.wrist1Angle=angle.angle1;
    this.wrist2Angle=angle.angle2;
  }

  /**
   * set the angle of the wrist for the state with raw angles
   * @param angle1
   *    the desired angle of the first joint
   * @param angle2
   *    the desired angle of the second joint
   */
  public void setWristAngle(double angle1, double angle2){
    this.wrist1Angle=angle1;
    this.wrist2Angle=angle2;
  }

  /**
   * set the status of the hatch intake for the state
   * @param open
   *    if the hatch intake is open
   */
  public void setHIntakeOpen(boolean open){
    this.hIntakeOpen=open;
  }

  /**
   * updates the state to the current positions of each part
   */
  public void updateToCurrent(){
    //TODO make seperate wrist angles, change intakeOpen from public boolean to get() function in Intake.java
    this.elevatorHeight = Superstructure.elevator.getHeight();
    this.wrist1Angle= Superstructure.wrist.getAngle();
    this.wrist2Angle= Superstructure.wrist.getAngle();
    this.hIntakeOpen= Robot.intakeOpen;
  }


  public double getElevatorHeight(){
    return this.elevatorHeight;
  }

  public double getWrist1Angle(){
    return this.wrist1Angle;
  }

  public double getWrist2Angle(){
    return this.wrist2Angle;
  }

  public boolean getHIntakeOpen(){
    return this.hIntakeOpen;
  }
}