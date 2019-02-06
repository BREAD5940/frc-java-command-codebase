package frc.robot.commands.auto;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;

import frc.robot.Robot;
import frc.robot.commands.auto.AutoMotion.GoalHeight;
import frc.robot.commands.auto.AutoMotion.GoalType;
import frc.robot.commands.auto.AutoMotion.HeldPiece;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;

/**
 * Creates an AutoMotion and drive plan based on the inputted params. Will
 * probably be used only in sandstorm. (and yes, this is basically 2018's auto
 * selector, but slightly different)
 */
public class AutoCombo {
  private AutoCommandGroup mBigCommandGroup;


  /**
   * generates the command groups based on the inputted goal height/type
   * @param startingPiece the game piece that the robot starts with (for example, a hatch)
   * @param wpKeys the keys for the waypoints (defined in Trajectories) for toe robot to go to.
   */

  public AutoCombo (HeldPiece startingPiece, String... wpKeys){
    HeldPiece cPiece = startingPiece; //the current piece is the piece we're starting with
    Pose2d cGoal =Trajectories.locations.get(wpKeys[0]); //goal init to unimportant value
    Pose2d cStart=Trajectories.locations.get(wpKeys[0]); //start init to the first waypoint
    
    for(int i=1; i<wpKeys.length-1; i++){ //starts at 1 so we don't get the current start
      cGoal = Trajectories.locations.get(wpKeys[i]); //goal to the current waypoint
      TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generatedTrajectories.get(new Pose2d[] {cStart,cGoal}); //current trajectory from hashmap in Trajectories
      AutoMotion cMotion = switchMotion(cPiece,wpKeys[i]); //creates an automotion based on the heldpiece and the goal
      cPiece = cMotion.getmHeldPiece(); //get the current heldpiece from the motion (at least for testing)
      traject=FieldConstraints.makeSafe(traject);//moves the trajectory so it doesn't hit stuff

      this.mBigCommandGroup.addSequential(Robot.drivetrain.followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal
      
      this.mBigCommandGroup.addSequential(cMotion.getBigCommandGroup()); //do a motion
      
      //prep for next loop
      cPiece = cMotion.getEndHeldPiece(); //set the current piece to the heldpiece at the end of the motion
      cStart = cGoal; //set the start to the current goal so it'll continue seamlessly
    }
    
  }
  
  /**
   * behold, one of the most terrible things i've ever created. it makes the automotion for the combo
   * @param piece
   *    the piece the robot is holding
   * @param goal
   *    the key for the goal location
   * @return
   *    an automotion
   */
  private AutoMotion switchMotion(HeldPiece piece, String goal){
    switch (goal.charAt(0)){ //gets the first letter of the goal key. they're all unique right now
      case 'h': //means it the hab
        System.out.println("Cannot have the hab platform as a goal"); //TODO do we _want_ to have the hab as a goal?
        return new AutoMotion(true); //TIXME this probably isn't the best way to have this, it's just empty
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
      case 'r': //rocket
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

  // id functions

  /**
   * identification function
   * @return
   *  the mBigCommandGroup of the function
   */
  public AutoCommandGroup getBigCommandGroup(){
    return this.mBigCommandGroup;
  }

  //not id functions
  
  
}
