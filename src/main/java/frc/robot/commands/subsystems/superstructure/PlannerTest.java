
package frc.robot.commands.subsystems.superstructure;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.planners.SuperstructureMotion;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class PlannerTest extends Command {
  SuperStructureState goal;
  public PlannerTest(SuperStructureState goal) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SuperstructureMotion.getInstance().plan(goal, SuperStructure.getInstance().lastState);
    SuperstructureMotion.getInstance().start();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true; // are you sure about that....
    // if this command exits will superstructureteleop do an oof?
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
