
package frc.robot.auto;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import frc.robot.Robot;
import frc.robot.auto.AutoPath.goals;
import frc.robot.auto.actions.auto_action_DRIVE;

public class AutoSelector{
    SendableChooser<AutoPath.robotLoc> rbLoc;
    SendableChooser<AutoPath.goals> usrGoal;
    SendableChooser<Integer> usrCubes;

    ArrayList<AutoPath> centerPaths;
    ArrayList<AutoPath> rightPaths;
    ArrayList<AutoPath> farRightPaths;
    ArrayList<AutoPath> leftPaths;
    ArrayList<AutoPath> farLeftPaths;

    AutoPath.robotLoc location;
    AutoPath.goals goal;
    Integer cubes;
    DriverStation ds;
    String fieldSetup;
    public AutoSelector(){
        //TODO this appears on smartdashboard without me having to do anything, right?
        rbLoc = new SendableChooser<AutoPath.robotLoc>();
        rbLoc.addDefault("Center", AutoPath.robotLoc.CENTER);
        rbLoc.addObject("Left", AutoPath.robotLoc.LEFT);
        rbLoc.addObject("Right", AutoPath.robotLoc.RIGHT);
        rbLoc.addObject("Far Right", AutoPath.robotLoc.FAR_RIGHT);
        rbLoc.addObject("Far Left", AutoPath.robotLoc.FAR_LEFT);

        usrGoal = new SendableChooser<AutoPath.goals>();
        usrGoal.addDefault("Near switch", goals.NEAR_SWITCH);
        usrGoal.addObject("Scale", goals.SCALE);
        usrGoal.addObject("Far switch", goals.FAR_SWITCH);

        usrCubes = new SendableChooser<Integer>();
        usrCubes.addDefault("0", 0);
        usrCubes.addObject("1", 1);
        usrCubes.addObject("2", 2);


        //TODO add actual command groups
        // TODO add multi-cube autos
        // Center paths
            centerPaths.add(new AutoPath("Right switch from center", AutoPath.goals.NEAR_SWITCH, AutoPath.robotLoc.CENTER, "RXX"));
            centerPaths.add(new AutoPath("Left switch from center", AutoPath.goals.NEAR_SWITCH, AutoPath.robotLoc.CENTER, "LXX"));
            centerPaths.add(new AutoPath("Right scale from center", AutoPath.goals.SCALE, AutoPath.robotLoc.CENTER, "XRX"));
            centerPaths.add(new AutoPath("Left scale from center", AutoPath.goals.SCALE, AutoPath.robotLoc.CENTER, "XLX"));
            centerPaths.add(new AutoPath("Right far switch from center", AutoPath.goals.FAR_SWITCH, AutoPath.robotLoc.CENTER, "XXR"));
            centerPaths.add(new AutoPath("Left far switch from center", AutoPath.goals.FAR_SWITCH, AutoPath.robotLoc.CENTER, "XXL"));

        // Left paths
            leftPaths.add(new AutoPath("Right switch from left", AutoPath.goals.NEAR_SWITCH, AutoPath.robotLoc.LEFT, "RXX"));
            leftPaths.add(new AutoPath("Left switch from left", AutoPath.goals.NEAR_SWITCH, AutoPath.robotLoc.LEFT, "LXX"));
            leftPaths.add(new AutoPath("Right scale from left", AutoPath.goals.SCALE, AutoPath.robotLoc.LEFT, "XRX"));
            leftPaths.add(new AutoPath("Left scale from left", AutoPath.goals.SCALE, AutoPath.robotLoc.LEFT, "XLX"));
            leftPaths.add(new AutoPath("Right far switch from left", AutoPath.goals.FAR_SWITCH, AutoPath.robotLoc.LEFT, "XXR"));
            leftPaths.add(new AutoPath("Left far switch from left", AutoPath.goals.FAR_SWITCH, AutoPath.robotLoc.LEFT, "XXL"));

        // Right paths
            rightPaths.add(new AutoPath("Right switch from right", AutoPath.goals.NEAR_SWITCH, AutoPath.robotLoc.RIGHT, "RXX"));
            rightPaths.add(new AutoPath("Left switch from right", AutoPath.goals.NEAR_SWITCH, AutoPath.robotLoc.RIGHT, "LXX"));
            rightPaths.add(new AutoPath("Right scale from right", AutoPath.goals.SCALE, AutoPath.robotLoc.RIGHT, "XRX"));
            rightPaths.add(new AutoPath("Left scale from right", AutoPath.goals.SCALE, AutoPath.robotLoc.RIGHT, "XLX"));
            rightPaths.add(new AutoPath("Right far switch from right", AutoPath.goals.FAR_SWITCH, AutoPath.robotLoc.RIGHT, "XXR"));
            rightPaths.add(new AutoPath("Left far switch from right", AutoPath.goals.FAR_SWITCH, AutoPath.robotLoc.RIGHT, "XXL"));

        // Far left paths
            farLeftPaths.add(new AutoPath("Right switch from far left", AutoPath.goals.NEAR_SWITCH, AutoPath.robotLoc.FAR_LEFT, "RXX"));
            farLeftPaths.add(new AutoPath("Left switch from far left", AutoPath.goals.NEAR_SWITCH, AutoPath.robotLoc.FAR_LEFT, "LXX"));
            farLeftPaths.add(new AutoPath("Right scale from far left", AutoPath.goals.SCALE, AutoPath.robotLoc.FAR_LEFT, "XRX"));
            farLeftPaths.add(new AutoPath("Left scale from far left", AutoPath.goals.SCALE, AutoPath.robotLoc.FAR_LEFT, "XLX"));
            farLeftPaths.add(new AutoPath("Right far switch from far left", AutoPath.goals.FAR_SWITCH, AutoPath.robotLoc.FAR_LEFT, "XXR"));
            farLeftPaths.add(new AutoPath("Left far switch from far left", AutoPath.goals.FAR_SWITCH, AutoPath.robotLoc.FAR_LEFT, "XXL"));

        // Far right paths
            farRightPaths.add(new AutoPath("Right switch from far right", AutoPath.goals.NEAR_SWITCH, AutoPath.robotLoc.FAR_RIGHT, "RXX"));
            farRightPaths.add(new AutoPath("Left switch from far right", AutoPath.goals.NEAR_SWITCH, AutoPath.robotLoc.FAR_RIGHT, "LXX"));
            farRightPaths.add(new AutoPath("Right scale from far right", AutoPath.goals.SCALE, AutoPath.robotLoc.FAR_RIGHT, "XRX"));
            farRightPaths.add(new AutoPath("Left scale from far right", AutoPath.goals.SCALE, AutoPath.robotLoc.FAR_RIGHT, "XLX"));
            farRightPaths.add(new AutoPath("Right far switch from far right", AutoPath.goals.FAR_SWITCH, AutoPath.robotLoc.FAR_RIGHT, "XXR"));
            farRightPaths.add(new AutoPath("Left far switch from far right", AutoPath.goals.FAR_SWITCH, AutoPath.robotLoc.FAR_RIGHT, "XXL"));
    }
    

    /**
     * selects the best autopath for the game
     * all inputs from sendable chooser
     */
    public AutoPath choosePath() {
        // String order is: near switch, scale, far switch
        this.fieldSetup = ds.getGameSpecificMessage();
        this.location = rbLoc.getSelected();
        this.goal = usrGoal.getSelected();
        this.cubes = usrCubes.getSelected();
        AutoPath defaultPath = new AutoPath("Default", AutoPath.goals.TEST, this.location, fieldSetup);

        switch (this.location){
            case CENTER:
                checkSetup(centerPaths);
            case LEFT:

            case RIGHT:

            case FAR_LEFT:

            case FAR_RIGHT:

            default:
                break;
        }
        

        return defaultPath;
    }

    // TODO finish
    private ArrayList<AutoPath> checkSetup(ArrayList<AutoPath> possiblePaths){
        ArrayList<AutoPath> goodPaths = new ArrayList<AutoPath>();
        for (AutoPath path : possiblePaths){
            if (path.getReqSetup().charAt(0) == this.fieldSetup.charAt(0)
                    &&path.getReqSetup().charAt(1)=='X'
                    &&path.getReqSetup().charAt(2)=='X'){
                goodPaths.add(path);
            }else if (path.getReqSetup().charAt(0)=='X'
                        &&path.getReqSetup)

        }
        return goodPaths;
    }
}