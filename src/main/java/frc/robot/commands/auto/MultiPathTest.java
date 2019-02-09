package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.lib.Logger;
import frc.robot.subsystems.DriveTrain;

public class MultiPathTest extends AutoCommandGroup {
  public MultiPathTest(){
    addSequential(DriveTrain.getInstance().followTrajectory(Trajectories.generatedTrajectories.get("test"), true));
    addSequential(DriveTrain.getInstance().followTrajectory(Trajectories.generatedTrajectories.get("test1"), false));
  }
}