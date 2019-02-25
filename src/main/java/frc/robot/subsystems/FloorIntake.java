package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.lib.RotatingSpark;

/**
 * Add your docs here.
 */
public class FloorIntake extends Subsystem {

  RotatingSpark mSpark;

  public FloorIntake(int pwmPort) {
    
  }




  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
