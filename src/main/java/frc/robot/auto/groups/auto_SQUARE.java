// all of this is Wrong and False plz ignore

package frc.robot.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auto.actions.auto_DRIVE;
import frc.robot.auto.actions.auto_TURN;

public class auto_SQUARE extends CommandGroup{

    public auto_SQUARE(double sideLength) {
        addSequential(new auto_DRIVE(sideLength));
        addSequential(new auto_TURN(3)); 
    }
}