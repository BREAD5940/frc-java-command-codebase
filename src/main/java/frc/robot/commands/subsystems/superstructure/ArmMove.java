package frc.robot.commands.subsystems.superstructure;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.states.IntakeAngle;

public class ArmMove extends Command{

  public ArmMove(IntakeAngle state){

  }

  @Override
  protected boolean isFinished() {
    return false;
  }

}