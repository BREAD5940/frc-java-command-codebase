package frc.robot.commands.auto;

import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.subsystems.DriveTrain;

public class MultiPathTest extends AutoCommandGroup {
	public MultiPathTest() {
		addSequential(DriveTrain.getInstance().followTrajectory(Trajectories.generatedHGTrajectories.get("test"), true));
		addSequential(DriveTrain.getInstance().followTrajectory(Trajectories.generatedHGTrajectories.get("test1"), false));
	}
}
