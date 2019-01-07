package frc.robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.auto.actions.auto_Elevator;

public class PrepareIntake extends CommandGroup {
    public PrepareIntake(ElevatorHeight height) {
        Robot.intake.setSpeed(0);
        addParallel( new auto_Elevator(demand, isInstant) )
    }
}