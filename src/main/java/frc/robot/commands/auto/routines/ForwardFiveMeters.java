package frc.robot.commands.auto.routines;

import java.util.ArrayList;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.auto.Trajectories;
import frc.robot.commands.subsystems.drivetrain.SetInitialOdometry;
import frc.robot.commands.subsystems.drivetrain.TrajectoryTrackerCommand;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;

public class ForwardFiveMeters extends CommandGroup {
  /**
   * Routines are a command group of paths used during autonomous. 
   * (side note: the only reason that I'm not using Auto Motion is because 
   * these routines are [for now] mostly sandstorm specific due to positioning
   * requirements)
   * Use them just like command groups
   */
  public ForwardFiveMeters() {
    
    if(Trajectories.forwardFiveMeters == null) {
      Trajectories.generateAllTrajectories();
    }

    TimedTrajectory<Pose2dWithCurvature> trajectory = Trajectories.forwardFiveMeters;

    addSequential(new SetInitialOdometry(trajectory));
    
    addSequential(Robot.drivetrain.followTrajectory(trajectory, TrajectoryTrackerMode.PUREPURSUIT, true) );

    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.
  }
}
