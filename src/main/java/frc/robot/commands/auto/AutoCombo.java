package frc.robot.commands.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.commands.auto.AutoMotion.GoalHeight;
import frc.robot.commands.auto.AutoMotion.GoalType;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;

/**
 * Creates an AutoMotion and drive plan based on the inputted params.
 * Will probably be used only in sandstorm. (and yes, this is basically
 * 2018's auto selector, but slightly different)
 */
public class AutoCombo {
  private AutoMotion motion;
  private CurrentLocation location;
  private GoalLocation goal;
  private AutoCommandGroup mBigCommandGroup;
  private DrivePlan[] drivePlans ;


  public enum CurrentLocation{
    HAB_1R, HAB_1M, HAB_1L, HAB_2R, HAB_2L, HAB_3, LS_R, LS_L
  }

  //TODO this is a really awful way to do this
  public enum GoalLocation{
    CARGO_LL,CARGO_LM,CARGO_LR,CARGO_RL,CARGO_RM,CARGO_RR,CARGO_ML,CARGO_MR,
    ROCKET_LL,ROCKET_LM,ROCKET_LR,ROCEKT_RL,ROCKET_RM,ROCKET_RR
  }


  /**
   * generates the command groups based on the inputted goal height/type
   * @param gHeight
   *    the height of the goal the robot should aim for (LOW, MIDDLE, HIGH, OVER)
   * @param gType
   *    the type of goal 
   * @param loc
   *    the current location of the robot
   */

  public AutoCombo (GoalHeight gHeight, GoalType gType, CurrentLocation loc, GoalLocation goal){
    this.motion = new AutoMotion(gHeight, gType);
    this.location = loc;
    this.goal = goal;
    this.mBigCommandGroup.addSequential(selectDrivePlan());
    this.mBigCommandGroup.addSequential(this.motion.getBigCommandGroup());
  }

  private Command selectDrivePlan(){
    ArrayList<DrivePlan> selectedDPs = new ArrayList<DrivePlan>();
    DrivePlan selected;
    for (int i=0; i<drivePlans.length; i++){
      if (drivePlans[i].goal==this.goal && drivePlans[i].start==this.location){
        selectedDPs.add(drivePlans[i]);
      }
    }
    //this just uses the first dp in the array TODO do we want to do some sort of additional selection

    // Am I reading this wrong? or right now selectDrivePlan() only returns one command? and then getBigCommandGroup returns only the
    // stuff necessary to place? Two hatch auto will have like a tone of stuff - drive -> place -> drive -> do a 180 while moving -> drive -> pickup hatch -> drive -> spin around while moving -> drive -> place
    // just a note to myself 
    return Robot.drivetrain.followTrajectory(selectedDPs.get(0).trajectory, TrajectoryTrackerMode.RAMSETE, true);
  }

  // id functions

  /**
   * identification function
   * @return
   *  the starting location of the combo
   */
  public CurrentLocation getLocation(){
    return this.location;
  }

  /**
   * identification function
   * @return
   *  the mBigCommandGroup of the function
   */
  public AutoCommandGroup getBigCommandGroup(){
    return this.mBigCommandGroup;
  }
  
}
