package frc.robot.commands.auto;

import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;

import frc.robot.commands.auto.AutoCombo.CurrentLocation;
import frc.robot.commands.auto.AutoCombo.GoalLocation;

public class DrivePlan{
  public TimedTrajectory<Pose2dWithCurvature> trajectory;
  public CurrentLocation start;
  public GoalLocation goal;

  public DrivePlan(TimedTrajectory<Pose2dWithCurvature> trajectory, CurrentLocation start, GoalLocation goal){
    this.trajectory = trajectory;
    this.start = start;
    this.goal = goal;
  }
}