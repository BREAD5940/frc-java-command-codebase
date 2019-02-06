package frc.robot.commands.auto;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.auto.AutoMotion.GoalHeight;
import frc.robot.commands.auto.AutoMotion.GoalType;
import frc.robot.commands.auto.AutoMotion.HeldPiece;


/**
 * Selects and runs an auto command group
 */
public class RunAuto extends Command {

  public GoalType gt;
  public GoalHeight height;
  public AutoMotion motion;
  public AutoCombo cMotion;
  public String[] keys;
  public boolean isDrive;
  public HeldPiece piece;


  public RunAuto(GoalType gt, GoalHeight height) {
    this.gt = gt;
    this.height = height;
    this.isDrive = false;
  }

  public RunAuto(HeldPiece piece, String... keys){
    this.keys = keys;
    this.isDrive = true;
    this.piece = piece;
  }

  @Override
  protected void initialize() {
    // Kinda a stupid question but what's the difference between AutoMotion and AutoCombo?
    // is it just that AutoMotion drives straight to a goal, whereas AutoCombo
    if(!isDrive){
      motion = new AutoMotion(height, gt);
      motion.getBigCommandGroup().start();
    }else{
      cMotion = new AutoCombo(piece, keys);
      cMotion.getBigCommandGroup().start();
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Don't need to do anything here
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return motion.getBigCommandGroup().done();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() { }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() { }
}
