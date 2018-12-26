
package frc.robot.auto;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.auto.actions.auto_action_DRIVE;

public class AutoSelector{

    

    /**
     * selects the best autopath for the game
     * @param location
     *      the current location of the robot (CENTER, LEFT, RIGHT, FARL, OR FARR)
     * @param fieldSetup
     *      the switch and scale assignments of the field (LLL, LLR, LRL, LRR, RRR, RLR, RLL, RRL)
     * @param goal
     *      the operator-selected goal from sendable chooser (nearSwitch, farSwitch, scale)
     * @param cubes
     *      the operator-selected number of cubes to place (1, 2, 3, etc.)
     */
    public AutoPath AutoSelector(AutoPath.robotLoc location, AutoPath.ssLoc fieldSetup, String goal, int cubes) {
        AutoPath idealPath = new AutoPath("Default", location, fieldSetup);

        

        return idealPath;
    }
}