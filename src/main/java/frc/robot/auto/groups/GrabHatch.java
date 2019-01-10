package frc.robot.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Command;

import java.util.ArrayList;

/** 
 * runs a series of commands to pick up a hatch from the loading station
 */
public class GrabHatch extends CommandGroup{
    public GrabHatch() {
        //run whatever commands make the robot intake a hatch
        /*
        Idea for what should happen: Get to known distance from goal and set the wrist, go forward
        to the distance such that the intake scrapes against the wall, intake while moving the elevator up,
        and once the elevator achives a target height start backing up.
        Start the elevator with a paralel command group, in which there is a waitForCondition command 
        followed sequentailly by a setdrivetrainspeeds command.
        */
    }

}