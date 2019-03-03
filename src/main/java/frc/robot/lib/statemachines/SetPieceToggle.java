package frc.robot.lib.statemachines;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.lib.statemachines.AutoMotionStateMachine.HeldPiece;

/**
 * is literally a toggle
 * 
 * @author jocleyn McHugo
 */
public class SetPieceToggle extends Command{
  public SetPieceToggle(){}

  @Override
  protected void initialize() {
    AutoMotionStateMachine.setHeldPiece(HeldPiece.CARGO);
  }

  @Override
  protected void interrupted(){
    AutoMotionStateMachine.setHeldPiece(HeldPiece.HATCH);
  }

  @Override
  protected void end(){
    interrupted();
  }

  @Override
  protected boolean isFinished() {
    return false;
  }
}