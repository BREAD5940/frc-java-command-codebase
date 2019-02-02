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
  private CurrentLocation location;
  private GoalLocation goal;
  private AutoCommandGroup mBigCommandGroup;
  private ArrayList<DrivePlan> drivePlans;

  public static HashMap<String,Pose2d> locations = new HashMap<String,Pose2d>();
  public static void genLocations(){
    locations.put("habR", new Pose2d(LengthKt.getFeet(5.106), LengthKt.getFeet(17.684),Rotation2dKt.getDegree(0.0)));
    locations.put("habM", new Pose2d(LengthKt.getFeet(5.181), LengthKt.getFeet(13.379),Rotation2dKt.getDegree(0.0)));
    locations.put("habL", new Pose2d(LengthKt.getFeet(5.141), LengthKt.getFeet(9.508),Rotation2dKt.getDegree(0.0)));
    locations.put("loadingL", new Pose2d(LengthKt.getFeet(5.106), LengthKt.getFeet(17.684),Rotation2dKt.getDegree(0.0)));
    locations.put("loadingR", new Pose2d(LengthKt.getFeet(5.106), LengthKt.getFeet(17.684),Rotation2dKt.getDegree(0.0)));
    locations.put("cargoLL", new Pose2d(LengthKt.getFeet(5.106), LengthKt.getFeet(17.684),Rotation2dKt.getDegree(0.0)));
    locations.put("cargoLM", new Pose2d(LengthKt.getFeet(5.106), LengthKt.getFeet(17.684),Rotation2dKt.getDegree(0.0)));
    locations.put("cargoLR", new Pose2d(LengthKt.getFeet(5.106), LengthKt.getFeet(17.684),Rotation2dKt.getDegree(0.0)));
    locations.put("cargoML", new Pose2d(LengthKt.getFeet(5.106), LengthKt.getFeet(17.684),Rotation2dKt.getDegree(0.0)));
    locations.put("cargoMR", new Pose2d(LengthKt.getFeet(5.106), LengthKt.getFeet(17.684),Rotation2dKt.getDegree(0.0)));
    locations.put("cargoRL", new Pose2d(LengthKt.getFeet(5.106), LengthKt.getFeet(17.684),Rotation2dKt.getDegree(0.0)));
    locations.put("cargoRM", new Pose2d(LengthKt.getFeet(5.106), LengthKt.getFeet(17.684),Rotation2dKt.getDegree(0.0)));
    locations.put("cargoRR", new Pose2d(LengthKt.getFeet(5.106), LengthKt.getFeet(17.684),Rotation2dKt.getDegree(0.0)));
    locations.put("rocketLL", new Pose2d(LengthKt.getFeet(5.106), LengthKt.getFeet(17.684),Rotation2dKt.getDegree(0.0)));
    locations.put("rocketLM", new Pose2d(LengthKt.getFeet(5.106), LengthKt.getFeet(17.684),Rotation2dKt.getDegree(0.0)));
    locations.put("rocketLR", new Pose2d(LengthKt.getFeet(5.106), LengthKt.getFeet(17.684),Rotation2dKt.getDegree(0.0)));
    locations.put("rocketRL", new Pose2d(LengthKt.getFeet(5.106), LengthKt.getFeet(17.684),Rotation2dKt.getDegree(0.0)));
    locations.put("rocketRM", new Pose2d(LengthKt.getFeet(5.106), LengthKt.getFeet(17.684),Rotation2dKt.getDegree(0.0)));
    locations.put("rocketRR", new Pose2d(LengthKt.getFeet(5.106), LengthKt.getFeet(17.684),Rotation2dKt.getDegree(0.0)));
  }
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
    return Robot.drivetrain.followTrajectory(selectedDPs.get(0).trajectory, TrajectoryTrackerMode.FEEDFORWARD, true);
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
