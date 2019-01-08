
package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;

import frc.robot.auto.AutoMotion;
import frc.robot.auto.AutoMotion.goalType;
import frc.robot.auto.AutoMotion.goalHeight;
import frc.robot.auto.AutoMotion.heldPiece;
import frc.robot.auto.groups.*;

public class AutoSelector{
  public SendableChooser<AutoMotion.heldPiece> hp;
  public SendableChooser<AutoMotion.goalHeight> gh;
  public SendableChooser<AutoMotion.goalType> gt;
  public SendableChooser<AutoMotion> backupAutoSelect = new SendableChooser<AutoMotion>();
  public boolean pathIsValid = true;


  ArrayList<AutoMotion> usableMotions = new ArrayList<AutoMotion>();
  ArrayList<AutoMotion> rocketMotions = new ArrayList<AutoMotion>();
  ArrayList<AutoMotion> cargoMotions = new ArrayList<AutoMotion>();

  AutoMotion defaultMotion = new AutoMotion("Default motion", goalHeight.LOW, goalType.CARGO_CARGO);
  goalHeight goalH;
  heldPiece piece;
  goalType goalT;

  /**
   * sets up required SendableChoosers
   * generates list of possible AutoMotions
   */
  public AutoSelector(){

    backupAutoSelect.setDefaultOption("Default Path", defaultMotion);

    /* TODO so this should really all be done with buttons for speed, but we don't have enough atm, so I'm putting it in sendable chooser for now*/


    gh = new SendableChooser<AutoMotion.goalHeight>();
    gh.setDefaultOption("Low", goalHeight.LOW);
    gh.addOption("Middle", goalHeight.MIDDLE);
    gh.addOption("High", goalHeight.HIGH);
    gh.addOption("Dropped into the cargo ship", goalHeight.OVER);

    gt = new SendableChooser<AutoMotion.goalType>();
    gt.setDefaultOption("Rocket Hatch", goalType.ROCKET_HATCH);
    gt.addOption("Cargo Ship Hatch", goalType.CARGO_HATCH);
    gt.addOption("Rocket Cargo", goalType.ROCKET_CARGO);
    gt.addOption("Cargo Ship Cargo", goalType.CARGO_CARGO);
    gt.addOption("Pick Up Hatch", goalType.RETRIEVE_HATCH);


    rocketMotions.add(new AutoMotion("Bottom level rocket cargo", goalHeight.LOW, goalType.ROCKET_CARGO));
    rocketMotions.add(new AutoMotion("Middle level rocket cargo", goalHeight.MIDDLE, goalType.ROCKET_CARGO));
    rocketMotions.add(new AutoMotion("Top level rocket cargo", goalHeight.HIGH, goalType.ROCKET_CARGO));

    rocketMotions.add(new AutoMotion("Bottom level rocket hatch", goalHeight.LOW, goalType.ROCKET_HATCH));
    rocketMotions.add(new AutoMotion("Middle level rocket hatch", goalHeight.MIDDLE, goalType.ROCKET_HATCH));
    rocketMotions.add(new AutoMotion("Top level rocket hatch", goalHeight.HIGH, goalType.ROCKET_HATCH));


    cargoMotions.add(new AutoMotion("Cargo ship direct cargo", goalHeight.LOW, goalType.CARGO_CARGO));
    cargoMotions.add(new AutoMotion("Cargo ship drop cargo", goalHeight.OVER, goalType.CARGO_CARGO));

    cargoMotions.add(new AutoMotion("Cargo ship hatch", goalHeight.LOW, goalType.CARGO_HATCH));

  }
  

  /**
   * selects the best AutoMotion for the current moment
   * @param goal
   *    the desired goal of the AutoMotion
   */
  public AutoMotion chooseMotion(goalType goal) {
    this.goalH = gh.getSelected();
    this.goalT = goal;
    if (goalT==goalType.CARGO_CARGO||goalT==goalType.ROCKET_CARGO){
      this.piece = heldPiece.CARGO;
    }else if (goalT==goalType.CARGO_HATCH||goalT==goalType.ROCKET_HATCH){
      this.piece = heldPiece.HATCH;
    }else{
      this.piece=heldPiece.NONE;
    }

    if (!(goalH==goalHeight.LOW || goalH == goalHeight.OVER)
        &&!(goalT==goalType.ROCKET_CARGO||goalT==goalType.ROCKET_HATCH)){
      System.out.println("You can't select those options together");
      this.pathIsValid = false;
    }
    else{
      this.pathIsValid = true;
    }
    SmartDashboard.putBoolean("Valid auto path?", pathIsValid);

    if(goalT==goalType.ROCKET_CARGO||goalT==goalType.ROCKET_HATCH){
      usableMotions=checkCompat(rocketMotions);
    }else if (goalT==goalType.CARGO_CARGO||goalT==goalType.CARGO_HATCH){
      usableMotions=checkCompat(cargoMotions);
    }else{
      // TODO put retrieve motions check here
    }
    
    if(usableMotions.size() !=1){
      try {
        return usableMotions.get(0);
      } catch (IndexOutOfBoundsException e) {
        //TODO: handle exception
        System.out.println("array out of bounds exception on chooseMotion");
        return defaultMotion;
      }

    }else{
      for (AutoMotion motion : usableMotions){
        backupAutoSelect.addOption(motion.getName(), motion);
      }
      // TODO find out if this just makes it select the default
      return backupAutoSelect.getSelected();
    }
    
  }

  /**
   * generates an ArrayList of AutoMotions that work with the current user inputs
   */
  private ArrayList<AutoMotion> checkCompat(ArrayList<AutoMotion> motions){
    ArrayList<AutoMotion> toReturn = new ArrayList<AutoMotion>();
    for(AutoMotion motion : motions){
      if (motion.getGoalHeight() == this.goalH && motion.getheldPiece() == this.piece){
        toReturn.add(motion);
      }
    }
    return toReturn;
  }

}
