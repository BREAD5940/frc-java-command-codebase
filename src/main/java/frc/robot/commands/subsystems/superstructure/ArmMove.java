package frc.robot.commands.subsystems.superstructure;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.states.IntakeAngle;
import frc.robot.subsystems.superstructure.SuperStructure;

public class ArmMove extends Command{

  IntakeAngle state;

  public ArmMove(IntakeAngle state){
    this.state = state;
    requires(SuperStructure.getInstance().getElbow());
    requires(SuperStructure.getInstance().getWrist());
  }

  @Override
  protected void initialize() {
    SuperStructure.getInstance().getWrist().requestAngle(state.wristAngle.angle);
    SuperStructure.getInstance().getElbow().requestAngle(state.elbowAngle.angle);
  }

  @Override
  protected boolean isFinished() {
    return Math.abs(state.wristAngle.angle.getDegree() - SuperStructure.getInstance().getWrist().getDegrees()) <= 2
            || Math.abs(state.elbowAngle.angle.getDegree() - SuperStructure.getInstance().getElbow().getDegrees()) <= 2;
  }

}