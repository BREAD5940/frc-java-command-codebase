
package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;

import frc.robot.auto.AutoMotion;
import frc.robot.auto.AutoMotion.goalType;
import frc.robot.auto.AutoMotion.goalHeight;
import frc.robot.auto.AutoMotion.heldPiece;
import frc.robot.auto.groups.*;
import frc.robot.Robot;

public class AutoSelector{
  public SendableChooser<AutoMotion.heldPiece> hp;
  public SendableChooser<AutoMotion.goalHeight> gh;
  public SendableChooser<AutoMotion.goalType> gt;
  public SendableChooser<AutoMotion> backupAutoSelect = new SendableChooser<AutoMotion>();
  public boolean pathIsValid = true;


  ArrayList<AutoMotion> usableMotions = new ArrayList<AutoMotion>();
  ArrayList<AutoMotion> rocketMotions = new ArrayList<AutoMotion>();
  ArrayList<AutoMotion> cargoMotions = new ArrayList<AutoMotion>();

  AutoMotion defaultMotion = new AutoMotion("Default motion", heldPiece.NONE, goalHeight.LOW, goalType.CARGO);
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

    hp = new SendableChooser<AutoMotion.heldPiece>();
    hp.setDefaultOption("None", heldPiece.NONE);
    hp.addOption("Hatch", heldPiece.HATCH);
    hp.addOption("Cargo", heldPiece.CARGO);

    gh = new SendableChooser<AutoMotion.goalHeight>();
    gh.setDefaultOption("Low", goalHeight.LOW);
    gh.addOption("Middle", goalHeight.MIDDLE);
    gh.addOption("High", goalHeight.HIGH);
    gh.addOption("Dropped into the cargo ship", goalHeight.OVER);

    gt = new SendableChooser<AutoMotion.goalType>();
    gt.setDefaultOption("Rocket", goalType.ROCKET);
    gt.addOption("Cargo Ship", goalType.CARGO);


    rocketMotions.add(new AutoMotion("Bottom level rocket cargo", heldPiece.CARGO, goalHeight.LOW, goalType.ROCKET));
    rocketMotions.add(new AutoMotion("Middle level rocket cargo", heldPiece.CARGO, goalHeight.MIDDLE, goalType.ROCKET));
    rocketMotions.add(new AutoMotion("Top level rocket cargo", heldPiece.CARGO, goalHeight.HIGH, goalType.ROCKET));

    rocketMotions.add(new AutoMotion("Bottom level rocket hatch", heldPiece.HATCH, goalHeight.LOW, goalType.ROCKET));
    rocketMotions.add(new AutoMotion("Middle level rocket hatch", heldPiece.HATCH, goalHeight.MIDDLE, goalType.ROCKET));
    rocketMotions.add(new AutoMotion("Top level rocket hatch", heldPiece.HATCH, goalHeight.HIGH, goalType.ROCKET));


    cargoMotions.add(new AutoMotion("Cargo ship direct cargo", heldPiece.CARGO, goalHeight.LOW, goalType.CARGO));
    cargoMotions.add(new AutoMotion("Cargo ship drop cargo", heldPiece.CARGO, goalHeight.OVER, goalType.CARGO));

    cargoMotions.add(new AutoMotion("Cargo ship hatch", heldPiece.HATCH, goalHeight.LOW, goalType.CARGO));

  }
  

  /**
   * selects the best AutoMotion for the game
   * all inputs from sendable chooser
   */
  public AutoMotion chooseMotion(heldPiece piece) {
    this.goalH = gh.getSelected();
    this.goalT = gt.getSelected();
    this.piece = piece;

    if (!(goalH==goalHeight.LOW || goalH == goalHeight.OVER)&&goalT!=goalType.ROCKET){
      System.out.println("You can't select those options together");
      this.pathIsValid = false;
    }
    else{
      this.pathIsValid = true;
    }
    SmartDashboard.putBoolean("Valid auto path?", pathIsValid);

    switch (this.goalT){
      case ROCKET:
        usableMotions = checkCompat(rocketMotions);
      case CARGO:
        usableMotions = checkCompat(cargoMotions);
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
