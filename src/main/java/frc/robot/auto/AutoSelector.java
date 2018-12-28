
package frc.robot.auto;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.auto.actions.auto_action_DRIVE;

public class AutoSelector{
    SendableChooser<AutoPath.robotLoc> rbLoc;
    AutoPath.robotLoc location;
    public AutoSelector(){
        rbLoc = new SendableChooser<AutoPath.robotLoc>();
        rbLoc.addDefault("Center", AutoPath.robotLoc.CENTER);
        rbLoc.addObject("Left", AutoPath.robotLoc.LEFT);
        rbLoc.addObject("Right", AutoPath.robotLoc.RIGHT);
        rbLoc.addObject("Far Right", AutoPath.robotLoc.FAR_RIGHT);
        rbLoc.addObject("Far Left", AutoPath.robotLoc.FAR_LEFT);


        
    }
    

    /**
     * selects the best autopath for the game
     * @param fieldSetup
     *      the switch and scale assignments of the field (LLL, LLR, LRL, LRR, RRR, RLR, RLL, RRL)
     * @param goal
     *      the operator-selected goal from sendable chooser (nearSwitch, farSwitch, scale)
     * @param cubes
     *      the operator-selected number of cubes to place (1, 2, 3, etc.)
     */
    public AutoPath choosePath(AutoPath.ssLoc fieldSetup, String goal, int cubes) {
        this.location = rbLoc.getSelected();
        AutoPath defaultPath = new AutoPath("Default", this.location, fieldSetup);

        

        return defaultPath;
    }
}