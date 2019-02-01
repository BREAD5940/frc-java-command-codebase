package frc.robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.subsystems.elevator.SetElevatorHeight;
import frc.robot.commands.subsystems.wrist.SetWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorPresets;

public class PrepareIntake extends CommandGroup {
    public PrepareIntake(ElevatorPresets height) {
        addParallel( new SetElevatorHeight(Elevator.getHeightEnumValue(height), false) );
        addParallel( new SetWrist(90, false) );
    }
}
