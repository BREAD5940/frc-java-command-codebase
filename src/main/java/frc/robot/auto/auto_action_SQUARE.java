// all of this is Wrong and False plz ignore

package frc.robot.auto;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.auto.auto_action_DRIVE;

CommandGroup auto_action_SQUARE{

    public auto_action_SQUARE() {
        addSequential(new auto_action_DRIVE(12,"high",5,30));
        addSequential(new auto_action_TURN()); 
    }
}