package frc.robot.commands.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.subsystems.intake.AutoIntake;


public class DropCargo extends CommandGroup{
  /**
   * drops cargo. 
   * if it's being shot through a hatch/port, it outtakes forward. otherwise it outtakes downward
   * @param isDrop
   *    if the cargo is being dropped into the cargo ship or not
   */
  public DropCargo(boolean isDrop) {
    if (!isDrop){
      // addSequential(new SetWrist(0, false));
      addSequential(new AutoIntake(-1, 5));
    }else{
      // addSequential(new SetWrist(-90, false)); // TODO the entire SetWrist command is currently commented out
      addSequential(new AutoIntake(-1, 5));
    }
    
  }

}
