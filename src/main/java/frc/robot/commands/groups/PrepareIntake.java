package frc.robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.auto.actions.auto_Elevator;
import frc.robot.commands.SetWrist;
import frc.robot.subsystems.Elevator.ElevatorHeight;

public class PrepareIntake extends CommandGroup {
    public PrepareIntake(ElevatorHeight height) {
        Robot.intake.setSpeed(0);
        addParallel( new auto_Elevator(Robot.elevator.getHeightEnumValue(ElevatorHeight.CARGO_SHIP_HATCH), true) );
        addParallel( new SetWrist(90) );
    }
}