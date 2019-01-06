
package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import frc.robot.auto.AutoMotion.startingPiece;
import frc.robot.auto.groups.*;

public class AutoSelector{
    public SendableChooser<AutoMotion.startingPiece> sp;
    public SendableChooser<Integer> usrCubes;
    public SendableChooser<AutoMotion> backupAutoSelect = new SendableChooser<AutoMotion>();

    ArrayList<AutoMotion> centerPaths;
    ArrayList<AutoMotion> rightPaths;
    ArrayList<AutoMotion> farRightPaths;
    ArrayList<AutoMotion> leftPaths;
    ArrayList<AutoMotion> farLeftPaths;
    ArrayList<AutoMotion> usablePaths;

    AutoMotion.startingPiece sPiece;
    Integer cubes;
    DriverStation ds;
    String fieldSetup;
    AutoMotion defaultPath;
    public AutoSelector(){

        backupAutoSelect.addDefault("Default Path", defaultPath);

        sp = new SendableChooser<AutoMotion.startingPiece>();
        sp.addDefault("None", startingPiece.NONE);
        sp.addObject("Hatch", startingPiece.HATCH);
        sp.addObject("Cargo", startingPiece.CARGO);


    }
    

    /**
     * selects the best AutoMotion for the game
     * all inputs from sendable chooser
     */
    public AutoMotion choosePath() {
        // String order is: near switch, scale, far switch
        this.fieldSetup = ds.getGameSpecificMessage();
        this.location = rbLoc.getSelected();
        this.goal = usrGoal.getSelected();
        this.cubes = usrCubes.getSelected();
        

        switch (this.location){
            case CENTER:
                usablePaths = checkCompat(checkSetup(centerPaths));
            case LEFT:
                usablePaths = checkCompat(checkSetup(leftPaths));
            case RIGHT:
                usablePaths = checkCompat(checkSetup(rightPaths));
            case FAR_LEFT:
                usablePaths = checkCompat(checkSetup(farLeftPaths));
            case FAR_RIGHT:
                usablePaths = checkCompat(checkSetup(farRightPaths));
            default:
                usablePaths.add(this.defaultPath);
        }
        if(usablePaths.size()<=1){
            return usablePaths.get(0);
        }else{
            for (AutoMotion path : usablePaths){
                backupAutoSelect.addObject(path.getName(), path);
            }
            // TODO find out if this just makes it select the default
            return backupAutoSelect.getSelected();
        }
        
    }

    private ArrayList<AutoMotion> checkCompat(ArrayList<AutoMotion> paths){
        ArrayList<AutoMotion> toReturn = new ArrayList<AutoMotion>();
        for(AutoMotion path : paths){
            if (path.getGoal() == this.goal){
                toReturn.add(path);
            }
        }
        return toReturn;
    }


    private ArrayList<AutoMotion> checkSetup(ArrayList<AutoMotion> possiblePaths){
        ArrayList<AutoMotion> goodPaths = new ArrayList<AutoMotion>();
        for (AutoMotion path : possiblePaths){
            if ((path.getReqSetup().charAt(0) == this.fieldSetup.charAt(0) || path.getReqSetup().charAt(0)=='X')
                    &&(path.getReqSetup().charAt(1)==this.fieldSetup.charAt(1) || path.getReqSetup().charAt(1)=='X')
                    &&(path.getReqSetup().charAt(2)==this.fieldSetup.charAt(2) || path.getReqSetup().charAt(2)=='X')){
                goodPaths.add(path);
            }

        }
        return goodPaths;
    }
}