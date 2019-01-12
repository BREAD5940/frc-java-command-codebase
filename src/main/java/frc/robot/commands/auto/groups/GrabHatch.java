package frc.robot.commands.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.subsystems.elevator.SetElevatorHeight;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Elevator.ElevatorPresets;

import java.util.ArrayList;

/** 
 * runs a series of commands to pick up a hatch from the loading station
 */
public class GrabHatch extends CommandGroup{
    public GrabHatch() {
        /* The plan right now is to lower the elevator, drive to
         a distance from the loading station based on the Lidar, 
         move the elevator up while intaking and once up to a 
        set height start slowly backing up.
        */
        addSequential(new SetElevatorHeight(ElevatorPresets.CARGO_SHIP_HATCH.getValue(), true));
        
    }

}