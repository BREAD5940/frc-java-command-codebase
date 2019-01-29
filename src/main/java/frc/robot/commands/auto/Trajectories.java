package frc.robot.commands.auto;

import java.util.Arrays;
import java.util.List;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d;
import org.ghrobotics.lib.mathematics.twodim.trajectory.TrajectoryGeneratorKt;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.TimingConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.VelocityLimitRegionConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import frc.robot.lib.Logger;

@SuppressWarnings("WeakerAccess")
public class Trajectories {
  private static List<TimingConstraint<Pose2dWithCurvature>> constraints = Arrays.asList(
          new CentripetalAccelerationConstraint(AccelerationKt.getAcceleration(LengthKt.getFeet(4.5))),
          new VelocityLimitRegionConstraint(new Rectangle2d(7.0, 0.0, 8.0, 13.0), VelocityKt.getVelocity(LengthKt.getFeet(3.0)))
  );
  
  public static TimedTrajectory<Pose2dWithCurvature> forwardFiveMeters, frontRightCargo, frontLeftCargo;

  public static List<Pose2d> pathFrontRightCargo = Arrays.asList(
    new Pose2d(LengthKt.getFeet(2), LengthKt.getFeet(10), Rotation2dKt.getDegree(0)),
    new Pose2d(LengthKt.getFeet(17), LengthKt.getFeet(12.5), Rotation2dKt.getDegree(0))
  );

  public static List<Pose2d> pathForwardFiveMeters = Arrays.asList(
      new Pose2d(LengthKt.getFeet(5f), LengthKt.getFeet(15f), Rotation2dKt.getDegree(0f)),
      new Pose2d(LengthKt.getFeet(11f), LengthKt.getFeet(15f), Rotation2dKt.getDegree(0f))
    );

  public static void generateAllTrajectories(){
    frontRightCargo = generateTrajectory(
      pathFrontRightCargo,
      false
    );
    forwardFiveMeters = generateTrajectory(
      pathForwardFiveMeters,
      false
    );
    Logger.log("Trajectories generated");
  }

  public static TimedTrajectory<Pose2dWithCurvature> generateTrajectory(List<Pose2d> waypoints, boolean reversed){
    return TrajectoryGeneratorKt.getDefaultTrajectoryGenerator().generateTrajectory(
          waypoints,
          constraints,
          VelocityKt.getVelocity(LengthKt.getFeet(0)),
          VelocityKt.getVelocity(LengthKt.getFeet(0)),
          VelocityKt.getVelocity(LengthKt.getFeet(10)),
          AccelerationKt.getAcceleration(LengthKt.getFeet(4.0)),
          reversed
    );
  }
}