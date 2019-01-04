//wow this is Big Stupid
package frc.robot.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;

import frc.robot.auto.actions.*;

public class auto_TestDrive extends CommandGroup{

    /**
     * move forward 3
     */
    public auto_TestDrive(){
        addSequential(new auto_DriveDistance(3));
    }
}