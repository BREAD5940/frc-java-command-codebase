package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotConfig;
import frc.robot.lib.SuperstructurePlanner;
import frc.robot.states.SuperstructureState;
import frc.robot.subsystems.superstructure.*;

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
  private static final double defaultHeight = RobotConfig.Superstructure.minElevatorHeight;
  private static final Wrist.WristPos defaultAngle = Wrist.WristPos.DEFAULT;
  
  public Superstructure(){}


  private SuperstructureState mCurrentState = new SuperstructureState();

  /**
   * plans the motion of the superstructure based on the current reqState and currentState of the superstructure.
   * should be called periodically while the robot is running
   */
  public void planSuperstructure(){
    
    mCurrentState.updateToCurrent();   
    
    if(!(mReqState==mCurrentState)){
      this.mCurrentCommandGroup = planner.plan(mReqState, mCurrentState);
    }
  
  }

  public CommandGroup moveSuperstructureCombo(double height, double angle1, double angle2, boolean openIntake){
    mCurrentState.updateToCurrent();
    mReqState.setElevatorHeight(height);
    mReqState.setWristAngle(angle1, angle2);
    mReqState.setHIntakeOpen(openIntake);

    planSuperstructure();

    return this.mCurrentCommandGroup;
  }

  public CommandGroup moveSuperstructureElevator(double height){
    return this.moveSuperstructureCombo(height, mCurrentState.getWrist1Angle(), mCurrentState.getWrist2Angle(), false);
  }

  public CommandGroup moveSuperstructureWrist(Wrist.WristPos angle){
    return this.moveSuperstructureCombo(defaultHeight, angle.angle1, angle.angle2, false);
  }

  public CommandGroup moveSuperstructureIntake(boolean openIntake){
    return this.moveSuperstructureCombo(defaultHeight, defaultAngle.angle1, defaultAngle.angle2, openIntake);
  }


  @Override
  protected void initDefaultCommand() {
    // pretty sure it doesn't need a default command, so leaving this empty
  }

}