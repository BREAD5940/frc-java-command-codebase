//wow this is Big Stupid
package frc.robot.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;

import frc.robot.auto.AutoPath;
import frc.robot.auto.actions.*;

public class auto_PassLine extends CommandGroup{

    /**
     * just crosses the auto line
     * @param location
     *      the location of the robot
     */
    public auto_PassLine(AutoPath.robotLoc location){
        if (location == AutoPath.robotLoc.FAR_LEFT || location == AutoPath.robotLoc.FAR_RIGHT){
            addSequential(new auto_DriveDistance(11));
        }else if (location == AutoPath.robotLoc.RIGHT){

        }else if (location == AutoPath.robotLoc.LEFT){

        }else{

        }
    }
}