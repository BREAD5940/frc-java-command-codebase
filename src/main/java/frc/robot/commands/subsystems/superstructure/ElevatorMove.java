package frc.robot.commands.subsystems.superstructure;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.states.ElevatorState;

public class ElevatorMove extends Command{

  public ElevatorMove(ElevatorState goal){

  }

  @Override
  protected boolean isFinished() {
    return false;
  }

}