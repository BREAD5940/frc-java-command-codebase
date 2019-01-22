package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.subsystems.elevator.SetElevatorHeight;

/** 
 * runs a series of commands to pick up a cargo from the ground
 */
public class GrabCargo extends CommandGroup{
    public GrabCargo() {
        addSequential(Robot.superstructure.moveSuperstructureElevator(0));
        // TODO intake cargo
    }

}