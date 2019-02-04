package frc.robot.commands.auto;

import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;


public class DrivePlan{
  public TimedTrajectory<Pose2dWithCurvature> trajectory;
  public Pose2d start;
  public Pose2d goal;

  public DrivePlan(TimedTrajectory<Pose2dWithCurvature> trajectory, Pose2d start, Pose2d goal){
    this.trajectory = trajectory;
    this.start = start;
    this.goal = goal;
  }
}