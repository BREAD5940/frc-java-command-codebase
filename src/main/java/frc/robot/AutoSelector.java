
package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.auto.auto_action_DRIVE;

public class AutoSelector{

    public enum robotLoc{
        CENTER, LEFT, RIGHT, FARL, FARR
    }

    public enum ssLoc{
        LLL, LLR, LRL, LRR, RRR, RLR, RLL, RRL
    }

    /**
     * selects the best autopath for the game
     * @param location
     *      the current location of the robot (CENTER, LEFT, RIGHT, FARL, OR FARR)
     * @param fieldSetup
     *      the switch and scale assignments of the field (LLL, LLR, LRL, LRR, RRR, RLR, RLL, RRL)
     */
    public CommandGroup AutoSelector(robotLoc location, ssLoc fieldSetup) {
        CommandGroup idealPath = new auto_action_SQUARE(); // oof this dont work y am i a failure

        return idealPath;
    }
}