package frc.robot.commands.auto.groups;

import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

/** 
 * runs a series of commands to pick up a cargo from the ground
 */
public class GrabCargo extends CommandGroup{
    public GrabCargo() {
        addSequential(Robot.superstructure.moveSuperstructureElevator(LengthKt.getInch(0)));
        // TODO intake cargo
    }

}