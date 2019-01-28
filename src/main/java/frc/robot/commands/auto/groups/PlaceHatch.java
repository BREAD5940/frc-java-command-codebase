package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.auto.actions.DriveDistance;
import frc.robot.commands.subsystems.intake.CloseClamp;

/**
 * runs a series of commands to place a hatch on the rocket or cargo ship
 */
public class PlaceHatch extends CommandGroup{
  public PlaceHatch() {
    // Move forward constantly until timed out while outtaking
    addParallel(new DriveDistance(30, 10, 5)); // This should just keep going forward for like 5secs TODO test
    // Outtake hatch
    addParallel(new CloseClamp());
  }

}
