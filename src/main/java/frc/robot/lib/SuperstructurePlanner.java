package frc.robot.lib;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.subsystems.wrist.SetWrist;
import frc.robot.states.SuperstructureState;
import frc.robot.subsystems.Wrist.WristPos;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.commands.subsystems.elevator.*;
import frc.robot.commands.subsystems.intake.CloseClamp;
import frc.robot.commands.subsystems.intake.OpenClamp;

public class SuperstructurePlanner{

  public SuperstructurePlanner(){}

  //TODO add values for certain elevator positions (ex. the wrist can be <0 if the elevator is >10)

  //TODO get actual irl angles TODO make the names less horrible
  final double eleBottomAngle1 = -10; //The lowest angle1 at which the intake is above the bottom of the elevator
  final double eleBottomAngle2 = 0; //The lowest angle2 at which the intake is above the bottom of the elevator
  final double minUnCrashHeight=5; //min height of the elevator where the intake will never hit the ground

  final double crossbarHeight = 20;

  boolean intakeCrashable = false; //intake capable of hitting the ground
  boolean intakeAtRisk = false; //intake at risk of hitting the crossbar

  public CommandGroup plan(SuperstructureState goalState, SuperstructureState currentState){
    CommandGroup toReturn = new CommandGroup();

    if((currentState.getWrist1Angle()==WristPos.INSIDE_ELEVATOR.angle1 
        && currentState.getWrist2Angle()==WristPos.INSIDE_ELEVATOR.angle2)
        || (goalState.getWrist1Angle()==WristPos.INSIDE_ELEVATOR.angle1
        && goalState.getWrist2Angle()==WristPos.INSIDE_ELEVATOR.angle2)){
          intakeAtRisk=true;
    }

    if((goalState.getWrist1Angle()<=eleBottomAngle1 || goalState.getWrist2Angle()<=eleBottomAngle2)){
      intakeCrashable=true;
    }

    if(intakeAtRisk&&goalState.getElevatorHeight()>=crossbarHeight&&currentState.getElevatorHeight()<=crossbarHeight){
      toReturn.addSequential(new SetWrist(WristPos.OUTSIDE_ELEVATOR)); //Keeps intake outside the elevator so it doesn't hit the crossbar
    }else{
      intakeAtRisk=false;
    }
    
    if (goalState.getElevatorHeight()<minUnCrashHeight&&intakeCrashable){
      toReturn.addSequential(new SetElevatorHeight(goalState.getElevatorHeight()));
      currentState.setElevatorHeight(goalState.getElevatorHeight());
      toReturn.addSequential(new SetWrist(eleBottomAngle1, eleBottomAngle2)); //TODO should probably change this to be less bad. something with circles
      currentState.setWristAngle(goalState.getWrist1Angle(), goalState.getWrist2Angle());
    }



    //checks done, fix any remaining errors

    if(currentState.getHIntakeOpen()!=goalState.getHIntakeOpen()){
      if(goalState.getHIntakeOpen()){
        toReturn.addSequential(new OpenClamp());
      }else{
        toReturn.addSequential(new CloseClamp());
      }
      currentState.setHIntakeOpen(goalState.getHIntakeOpen());
    }

    if(currentState.getElevatorHeight()!=goalState.getElevatorHeight()){
      toReturn.addSequential(new SetElevatorHeight(goalState.getElevatorHeight()));
      currentState.setElevatorHeight(goalState.getElevatorHeight());
    }

    if(currentState.getWrist1Angle()!=goalState.getWrist1Angle()){
      toReturn.addSequential(new SetWrist(goalState.getWrist1Angle()));
    }
    if(currentState.getWrist2Angle()!=goalState.getWrist2Angle()){
      toReturn.addSequential(new SetWrist(goalState.getWrist2Angle()));
    }
    currentState.setWristAngle(goalState.getWrist1Angle(), goalState.getWrist2Angle());


    if(currentState==goalState){
      return toReturn;
    }else{
      System.out.println("We done goofed");
      return null;
    }
  }
}