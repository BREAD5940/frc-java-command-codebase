package frc.robot.commands.auto;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d;
import org.ghrobotics.lib.mathematics.twodim.trajectory.TrajectoryGeneratorKt;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.CentripetalAccelerationConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.TimingConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.constraints.VelocityLimitRegionConstraint;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.lib.Logger;

@SuppressWarnings("WeakerAccess")
public class Trajectories {

  public static Velocity<Length> kDefaultStartVelocity = VelocityKt.getVelocity(LengthKt.getFeet(0));
  public static Velocity<Length> kDefaultEndVelocity = VelocityKt.getVelocity(LengthKt.getFeet(0));

  public static Velocity<Length> kDefaultVelocity = VelocityKt.getVelocity(LengthKt.getFeet(2));
  public static final Acceleration<Length> kDefaultAcceleration = AccelerationKt.getAcceleration(LengthKt.getFeet(4));


  private static List<TimingConstraint<Pose2dWithCurvature>> kDefaultConstraints = Arrays.asList(
    // This limits our centripetal acceleration to 3 feet per second per second
    new CentripetalAccelerationConstraint(AccelerationKt.getAcceleration(LengthKt.getFeet(8)))/*,
    // This limits our velocity while within the given Rectangle2d to 2 feet per second (read: the hab)
    // new VelocityLimitRegionConstraint(new Rectangle2d(7.0, 0.0, 8.0, 13.0), VelocityKt.getVelocity(LengthKt.getFeet(2.0)))*/
  );

  public static HashMap<String, TimedTrajectory<Pose2dWithCurvature>> traject = new HashMap<String, TimedTrajectory<Pose2dWithCurvature>>();
  public static HashMap<String, List<Pose2d>> wpLists = new HashMap<String, List<Pose2d>>();
  
  public static void genLists(){
    wpLists.put("frontRightCargo",Arrays.asList(
      new Pose2d(LengthKt.getFeet(2), LengthKt.getFeet(10), Rotation2dKt.getDegree(0)),
      new Pose2d(LengthKt.getFeet(17), LengthKt.getFeet(12.5), Rotation2dKt.getDegree(0))
    ));
    wpLists.put("frontLeftCargo",null);
    wpLists.put("leftLeftCargo",Arrays.asList(
      ,
      new Pose2d(LengthKt.getFeet(21.747),LengthKt.getFeet(16.973),Rotation2dKt.getDegree(-91.0))
    ));
    wpLists.put("leftMiddleCargo",null);
    wpLists.put("leftRightCargo",null);
    wpLists.put("rightLeftCargo",null);
    wpLists.put("rightMiddleCargo",null);
    wpLists.put("rightRightCargo",null);
  }

  public static void generateAllTrajectories(){
    genLists();
    Logger.log("Generating ALL trajectories");
    double startTime = Timer.getFPGATimestamp();
    for (String key : wpLists.keySet()){
      traject.put(key,generateTrajectory(wpLists.get(key),false));
    }
    Logger.log("Trajectories generated in " + (Timer.getFPGATimestamp() - startTime) + "seconds!");
  }
  

  /**
   * Generate a trajectory from a list of waypoints
   * @param waypoints to follow
   * @param reversed if the path is reversed
   * @return
   */
  public static TimedTrajectory<Pose2dWithCurvature> generateTrajectory(List<Pose2d> waypoints, boolean reversed){
    return generateTrajectory(waypoints, kDefaultConstraints, kDefaultStartVelocity, kDefaultEndVelocity, kDefaultVelocity, kDefaultAcceleration, false);
  }

  /**
   * Generate a trajectory from scratch. Given a list of waypoints, constriants, velocities and accelerations, a trajectory will
   * be generated using the default trajectory generator.
   * @param waypoints to follow
   * @param constraints_ to respect (slow zone, acceleration, etc)
   * @param startVelocity to start at
   * @param endVelocity to end at
   * @param maxVelocity that will never be exceded
   * @param maxAcceleration for the trajectory
   * @param reversed for if the path should be reversed (flipped)
   */
  public static TimedTrajectory<Pose2dWithCurvature> generateTrajectory(List<Pose2d> waypoints, 
                                List<TimingConstraint<Pose2dWithCurvature>> constraints_, Velocity<Length> startVelocity, Velocity<Length> endVelocity, Velocity<Length> maxVelocity, Acceleration<Length> maxAcceleration, boolean reversed){
    return TrajectoryGeneratorKt.getDefaultTrajectoryGenerator().generateTrajectory(
          waypoints,
          constraints_,
          startVelocity,
          endVelocity,
          maxVelocity,
          maxAcceleration,
          reversed
    );
  }

}