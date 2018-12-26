
package frc.robot.auto;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Robot;
import frc.robot.auto.actions.auto_DriveDistance;

import frc.robot.auto.AutoPath;

public class AutoSelector{

    SendableChooser<AutoPath.robotLoc> robotLoc;
	// AutoPath.robotLoc prevRobotLoc = null;
	ArrayList<AutoPath> autoPaths = new ArrayList<>();

	final ArrayList<AutoPath> totalPossiblePaths = new ArrayList<>();

	// final AutoPath emptyAction;

    RobotLocation = AutoPath.robotLoc;

	// public static final boolean ROBOT_AUTONOMOUS_WORKS = false;

    // TODO get fms game stuff
    
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
    public AutoSelector(AutoPath.robotLoc location, AutoPath.ssLoc fieldSetup, String goal, int cubes) {
        AutoPath idealPath = new AutoPath("Default", location, fieldSetup);

        


        // this.emptyAction = new AutoPath("Do Nothing", "XXX", RobotLocation.FAR_LEFT);

        // return idealPath;
    }
}