// all of this is Wrong and False plz ignore

package frc.robot.auto.actions;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.auto.actions.auto_action_DRIVE;

public class auto_action_SQUARE extends CommandGroup{

    public auto_action_SQUARE(double sideLength) {
        // addSequential(new auto_action_DRIVE(sideLength,"high",5,30));
        addSequential(new auto_action_TURN()); 
    }
}