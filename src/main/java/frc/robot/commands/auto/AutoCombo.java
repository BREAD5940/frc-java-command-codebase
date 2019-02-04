package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.HashMap;

import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;

import edu.wpi.first.wpilibj.command.Command;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
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
  private Pose2d location;
  private Pose2d goal;
  private AutoCommandGroup mBigCommandGroup;
  private ArrayList<DrivePlan> drivePlans;


  /**
   * generates the command groups based on the inputted goal height/type
   * @param gHeight
   *    the height of the goal the robot should aim for (LOW, MIDDLE, HIGH, OVER)
   * @param gType
   *    the type of goal 
   * @param loc
   *    the current location of the robot
   */

  public AutoCombo (GoalHeight gHeight, GoalType gType, Pose2d loc, Pose2d goal){

    this.motion = new AutoMotion(gHeight, gType);
    this.location = loc;
    this.goal = goal;
    genDrivePlans();
    this.mBigCommandGroup.addSequential(selectDrivePlan());
    this.mBigCommandGroup.addSequential(this.motion.getBigCommandGroup());
  }

  private Command selectDrivePlan(){
    ArrayList<DrivePlan> selectedDPs = new ArrayList<DrivePlan>();
    DrivePlan selected;
    for (int i=0; i<drivePlans.size(); i++){
      if (drivePlans.get(i).goal==this.goal && drivePlans.get(i).start==this.location){
        selectedDPs.add(drivePlans.get(i));
      }
    }
    //this just uses the first dp in the array TODO do we want to do some sort of additional selection

    // Am I reading this wrong? or right now selectDrivePlan() only returns one command? and then getBigCommandGroup returns only the
    // stuff necessary to place? Two hatch auto will have like a tone of stuff - drive -> place -> drive -> do a 180 while moving -> drive -> pickup hatch -> drive -> spin around while moving -> drive -> place
    // just a note to myself 
    return Robot.drivetrain.followTrajectory(selectedDPs.get(0).trajectory, TrajectoryTrackerMode.RAMSETE, true);
  }

  public void genDrivePlans(){
    drivePlans.add(new DrivePlan(Trajectories.traject.get("frontRightCargo"),))
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
