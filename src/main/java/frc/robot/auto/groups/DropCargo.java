package frc.robot.auto.groups;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.Command;

import frc.robot.auto.actions.*;
import frc.robot.subsystems.Intake;

import java.util.ArrayList;

public class DropCargo extends CommandGroup{
  /**
   * drops cargo. 
   * if it's being shot through a hatch/port, it outtakes at full speed. otherwise it outtakes at half speed
   * @param isDrop
   *    if the cargo is being dropped into the cargo ship or not
   */
  public DropCargo(boolean isDrop) {
    if (!isDrop){
      addSequential(new AutoIntake(-1, 5));
    }else{
      addSequential(new AutoIntake(-0.5, 5));
    }
    
  }

}
