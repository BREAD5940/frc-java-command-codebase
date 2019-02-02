package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.auto.AutoMotion.mHeldPiece;
import frc.robot.commands.subsystems.intake.AutoIntake;
import frc.robot.commands.auto.actions.SetIntakeMode;
import frc.robot.commands.subsystems.elevator.SetElevatorHeight;

/** 
 * runs a series of commands to pick up a cargo from the ground
 */
public class GrabCargo extends CommandGroup{
    public GrabCargo() {
        addSequential(new SetElevatorHeight(0, true));
        addSequential(new SetIntakeMode(mHeldPiece.CARGO, true));
        addSequential(new AutoIntake(1, 20)); // FIXME check runtime
    }

}