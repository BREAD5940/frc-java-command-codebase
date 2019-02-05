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
import frc.robot.commands.auto.AutoMotion.HeldPiece;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;
import kotlin.jvm.Throws;

/**
 * Creates an AutoMotion and drive plan based on the inputted params.
 * Will probably be used only in sandstorm. (and yes, this is basically
 * 2018's auto selector, but slightly different)
 */
public class AutoCombo {
  private AutoMotion motion;
  private String[] wpKeys;
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

  public AutoCombo (HeldPiece startingPiece, String... wpKeys){
    this.wpKeys = wpKeys;
    genDrivePlans();
    HeldPiece cPiece = startingPiece;
    Pose2d cGoal;
    Pose2d cStart=Trajectories.locations.get(wpKeys[0]);;
    for(int i=1; i<wpKeys.length; i++){
      cGoal = Trajectories.locations.get(wpKeys[i]);
      AutoMotion cMotion = switchMotion(startingPiece,wpKeys[i]);
      cPiece = cMotion.getmHeldPiece();
      this.mBigCommandGroup.addSequential(selectDrivePlan(cStart, cGoal));
      this.mBigCommandGroup.addSequential(cMotion.getBigCommandGroup());
      //prep for next loop
      cPiece = cMotion.getEndHeldPiece();
      cStart = cGoal;
    }
    
  }

  private Command selectDrivePlan(Pose2d start, Pose2d goal){
    ArrayList<DrivePlan> selectedDPs = new ArrayList<DrivePlan>();
    DrivePlan selected;
    for (int i=0; i<drivePlans.size(); i++){
      if (drivePlans.get(i).goal==goal && drivePlans.get(i).start==start){
        selectedDPs.add(drivePlans.get(i));
      }
    }
    //this just uses the first dp in the array TODO do we want to do some sort of additional selection

    // Am I reading this wrong? or right now selectDrivePlan() only returns one command? and then getBigCommandGroup returns only the
    // stuff necessary to place? Two hatch auto will have like a tone of stuff - drive -> place -> drive -> do a 180 while moving -> drive -> pickup hatch -> drive -> spin around while moving -> drive -> place
    // just a note to myself 
    return Robot.drivetrain.followTrajectory(selectedDPs.get(0).trajectory, TrajectoryTrackerMode.RAMSETE, true);
  }
  
  private AutoMotion switchMotion(HeldPiece piece, String goal){
    switch (goal.charAt(0)){
      case 'h': //means it the hab
        System.out.println("Cannot have the hab platform as a goal");
        return new AutoMotion(true);
      case 'l': //it the loading station
        if (piece==HeldPiece.NONE){
          return new AutoMotion(GoalHeight.LOW, GoalType.RETRIEVE_HATCH);
        }else{
          System.out.println("Cannot load while carrying cargo/hatches");
          return new AutoMotion(true);
        }
      case 'c': //cargo ship
        if (piece==HeldPiece.CARGO){
          return new AutoMotion(GoalHeight.LOW,GoalType.CARGO_CARGO);
        }else if (piece==HeldPiece.HATCH){
          return new AutoMotion(GoalHeight.LOW, GoalType.CARGO_HATCH);
        }else{
          System.out.println("Cannot deposit nonexistant cargo/hatches");
          return new AutoMotion(true);
        }
      case 'r': //o shit it dat rocket
        if (piece==HeldPiece.CARGO){
          return new AutoMotion(GoalHeight.LOW, GoalType.ROCKET_CARGO);
        }else if (piece==HeldPiece.HATCH){
          return new AutoMotion(GoalHeight.LOW, GoalType.ROCKET_HATCH);
        }else{
          System.out.println("Cannot deposit nonexistant cargo/hatches");
          return new AutoMotion(true);
        }
      case 'd': //might not be home but it sur is a depot
        if (piece==HeldPiece.NONE){
          return new AutoMotion(GoalHeight.LOW, GoalType.RETRIEVE_CARGO);
        }else{
          System.out.println("Cannot collect from depot while holidn hatch/cargo");
          return new AutoMotion(true);
        }
    }
    return new AutoMotion(true);
  }

  public void genDrivePlans(){
    // drivePlans.add(new DrivePlan(Trajectories.traject.get("frontRightCargo"),))
  }

  // id functions

  /**
   * identification function
   * @return
   *  the mBigCommandGroup of the function
   */
  public AutoCommandGroup getBigCommandGroup(){
    return this.mBigCommandGroup;
  }
  
}
