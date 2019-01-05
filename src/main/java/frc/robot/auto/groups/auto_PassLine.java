//wow this is Big Stupid
package frc.robot.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;

import frc.robot.auto.AutoPath;
import frc.robot.auto.actions.*;
import frc.robot.Robot;

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
            addSequential(new auto_TurnInPlace(-15)); // TODO test this value
            addSequential(new auto_DriveDistance(11));
        }else if (location == AutoPath.robotLoc.LEFT){
            addSequential(new auto_TurnInPlace(15)); // TODO test this value
            addSequential(new auto_DriveDistance(11));
        }else{
            // TODO find out if I'm actually doing this right (seems unlikely but you never know)
            addSequential(new auto_FollowMotionProfile("\\src\\main\\deploy\\paths\\CenterAcrossLine.left.pf1.csv", "\\src\\main\\deploy\\paths\\CenterAcrossLine.right.pf1.csv"));
        }

        addSequential(new auto_Elevator(15));
    }
}