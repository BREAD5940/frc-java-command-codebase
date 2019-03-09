package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Add your docs here.
 */
public class InstantRunnable extends InstantCommand {

  Runnable toRun;

  public InstantRunnable(Runnable thing) {
    super();
    this.toRun = thing;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    try {
      toRun.run();
    } catch (Exception e) {
      //TODO: handle exception
    }
  }

}
