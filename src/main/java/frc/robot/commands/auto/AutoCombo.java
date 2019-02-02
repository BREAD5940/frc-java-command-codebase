package frc.robot.commands.auto;

import frc.robot.commands.auto.AutoMotion.mGoalHeight;
import frc.robot.commands.auto.AutoMotion.mGoalType;
import frc.robot.commands.auto.groups.AutoCommandGroup;

/**
 * Creates an AutoMotion and drive plan based on the inputted params.
 * Will probably be used only in sandstorm
 * 
 * @author Jocelyn McHugo
 */
public class AutoCombo {
  private AutoMotion motion;
  private mCurrentLocation location;
  private AutoCommandGroup mBigCommandGroup;


  public enum mCurrentLocation{
    HAB_1R, HAB_1M, HAB_1L, HAB_2R, HAB_2L, HAB_3, LS_R, LS_L
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

  public AutoCombo (mGoalHeight gHeight, mGoalType gType, mCurrentLocation loc){
    this.motion = new AutoMotion(gHeight, gType);
    this.location = loc;
    this.mBigCommandGroup.addSequential(selectDrivePlan());
    this.mBigCommandGroup.addSequential(this.motion.getBigCommandGroup());
  }

  private AutoCommandGroup selectDrivePlan(){
    AutoCommandGroup toReturn = new AutoCommandGroup();
    //TODO drive plan selector here
    return toReturn;
  }

  // id functions

  /**
   * identification function
   * @return
   *  the starting location of the combo
   */
  public mCurrentLocation getLocation(){
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
