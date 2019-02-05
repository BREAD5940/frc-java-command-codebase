package frc.robot.commands.auto;

import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;

import edu.wpi.first.wpilibj.command.Command;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;

import frc.robot.Robot;
import frc.robot.RobotConfig;
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
  private ArrayList<TimedTrajectory<Pose2dWithCurvature>> drivePlans;


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
    HeldPiece cPiece = startingPiece; //the current piece is the piece we're starting with
    Pose2d cGoal =Trajectories.locations.get(wpKeys[0]); //goal init to unimportant value
    Pose2d cStart=Trajectories.locations.get(wpKeys[0]); //start init to the first waypoint
    
    for(int i=1; i<wpKeys.length-1; i++){ //starts at 1 so we don't get the current start
      cGoal = Trajectories.locations.get(wpKeys[i]); //goal to the current waypoint
      TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generatedTrajectories.get(new Pose2d[] {cStart,cGoal}); //current trajectory from hashmap in Trajectories
      AutoMotion cMotion = switchMotion(cPiece,wpKeys[i]); //creates an automotion based on the heldpiece and the goal
      cPiece = cMotion.getmHeldPiece(); //get the current heldpiece from the motion TODO do I actually need this?
      if(FieldConstraints.isSafe(traject)){ //checks if it's safe (wont hit shit)
        this.mBigCommandGroup.addSequential(Robot.drivetrain.followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, true)); //drive to goal
        this.mBigCommandGroup.addSequential(cMotion.getBigCommandGroup()); //do a motion
      }else{
        //TODO so isSafe should really really REALLY just fix the dam path
      }
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
  private static class FieldConstraints{
    protected static double rad = RobotConfig.auto.robotRadius.getFeet();
    protected static final Length maxY = LengthKt.getFeet(27-rad);
    protected static final Length minY = LengthKt.getFeet(0+rad);
    protected static final Length maxX = LengthKt.getFeet(54-rad);
    protected static final Length minX = LengthKt.getFeet(0+rad);

    protected static final Translation2d[] cargo = {new Translation2d(LengthKt.getFeet(18.208-rad), LengthKt.getFeet(15.386+rad)),
                                                    new Translation2d(LengthKt.getFeet(36.738+rad), LengthKt.getFeet(11.717-rad))};
    protected static final Translation2d[] rocketL = {new Translation2d(LengthKt.getFeet(17.602-rad), LengthKt.getFeet(27+rad)),
                                                      new Translation2d(LengthKt.getFeet(20.422+rad), LengthKt.getFeet(24.745-rad))};
    protected static final Translation2d[] rocketR = {new Translation2d(LengthKt.getFeet(17.602-rad), LengthKt.getFeet(2.134+rad)),
                                                      new Translation2d(LengthKt.getFeet(20.422+rad), LengthKt.getFeet(0-rad))};
    protected static final Translation2d[] upperHabaDepot = {new Translation2d(LengthKt.getFeet(0-rad), LengthKt.getFeet(21+rad)),
                                                              new Translation2d(LengthKt.getFeet(4+rad), LengthKt.getFeet(6-rad))};
    protected static final Translation2d[] habRamp = {new Translation2d(LengthKt.getFeet(4-rad), LengthKt.getFeet(20+rad)),
                                                      new Translation2d(LengthKt.getFeet(8+rad), LengthKt.getFeet(7.15-rad))};

    public static boolean isSafe(TimedTrajectory<Pose2dWithCurvature> traject){
      List<TimedEntry<Pose2dWithCurvature>> points = traject.getPoints();
      List<Translation2d[]> constraints = new ArrayList<Translation2d[]>(Arrays.asList(cargo,rocketL,rocketR,upperHabaDepot,habRamp));
      if(isOutsideField(traject)){
        return false; //see comment below
      }
      for(int j=0; j<constraints.size()-1; j++){
        for (int i=0; i<points.size()-1; i++){
          Translation2d point = points.get(i).getState().getPose().getTranslation();
          //translation array cycles topLeft->bottomRight
          if(!(point.getX().getFeet()>constraints.get(j)[0].getX().getFeet()&&point.getX().getFeet()<constraints.get(j)[1].getX().getFeet()
                &&point.getY().getFeet()>constraints.get(j)[0].getY().getFeet()&&point.getX().getFeet()<constraints.get(j)[1].getX().getFeet())){
            return false; //this is an if statement bc it loops again if it's not false. it's this or a while loop
            //TODO instead of returning a boolean, this should just straight-up fix the path
          }
        }
      }
      return true;
    }

    public static boolean isOutsideField(TimedTrajectory<Pose2dWithCurvature> traject){
      List<TimedEntry<Pose2dWithCurvature>> points = traject.getPoints();
      boolean bad = false;

      for(int i=0; i<points.size()-1; i++){
        Translation2d point = points.get(i).getState().getPose().getTranslation();
        bad=(point.getX().getFeet()>maxX.getFeet()
            || point.getX().getFeet()<minX.getFeet()
            || point.getY().getFeet()>maxY.getFeet()
            || point.getY().getFeet()<minY.getFeet());

        //TODO would it be bad^TM if this just set the x/y of the point to the max/min x/y?
      }
      return bad;
    }

    public static TimedTrajectory<Pose2dWithCurvature> makeSafe(TimedTrajectory<Pose2dWithCurvature> traject){
      List<TimedEntry<Pose2dWithCurvature>> points = traject.getPoints();
      List<TimedEntry<Pose2dWithCurvature>> safePoints = new ArrayList<TimedEntry<Pose2dWithCurvature>>();
      List<Translation2d[]> constraints = new ArrayList<Translation2d[]>(Arrays.asList(cargo,rocketL,rocketR,upperHabaDepot,habRamp));
      if(isSafe(traject)){
        System.out.println("Trajectory is already safe!");
        return traject;
      }

      for(int i=0; i<points.size()-1; i++){
        Translation2d point = points.get(i).getState().getPose().getTranslation();
        Length safeX = point.getX();
        Length safeY = point.getY();
        if(point.getX().getFeet()>maxX.getFeet()){safeX=maxX;}
        if(point.getX().getFeet()<minX.getFeet()){safeX=minX;}
        if(point.getY().getFeet()>maxY.getFeet()){safeY=maxY;}
        if(point.getY().getFeet()<minY.getFeet()){safeY=minY;}

        for(int j=0; j<constraints.size()-1; j++){
          if(point.getX().getFeet()<constraints.get(j)[0].getX().getFeet()&&point.getX().getFeet()>constraints.get(j)[1].getX().getFeet()){
            //TODO set safeX to the nearest x val on the border
          }
          if(point.getY().getFeet()>constraints.get(j)[0].getY().getFeet()&&point.getX().getFeet()<constraints.get(j)[1].getX().getFeet()){
            //TODO set safeY to the nearest y val on the border
          }
        
        }

        safePoints.add(i,new TimedEntry<Pose2dWithCurvature>((new Pose2dWithCurvature(new Pose2d(new Translation2d(safeX,safeY),points.get(i).getState().getPose().getRotation()),points.get(i).getState().getCurvature())),
                          points.get(i).getT(), points.get(i).getVelocity(), points.get(i).getAcceleration()));
      }

      //TODO add some sort of smoother
      return new TimedTrajectory<Pose2dWithCurvature>(safePoints, false);
    }
  }
  
}
