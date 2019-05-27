//package frc.robot.commands.auto;
//
//
//import frc.robot.subsystems.DriveTrain;
//import org.team5940.pantry.exparimental.command.SequentialCommandGroup;
//
//public class MultiPathTest extends SequentialCommandGroup {
//	public MultiPathTest() {
//		addSequential(DriveTrain.getInstance().followTrajectory(Trajectories.generatedHGTrajectories.get("test"), true));
//		addSequential(DriveTrain.getInstance().followTrajectory(Trajectories.generatedHGTrajectories.get("test1"), false));
//	}
//}
