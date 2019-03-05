package frc.robot.commands.subsystems.drivetrain;

import java.util.Arrays;
import java.util.List;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.auto.Trajectories;
import frc.robot.lib.AutoCommand;
import frc.robot.lib.motion.Util;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;

public class DriveDistanceTheSecond extends Command {

	private static final Velocity<Length> kCruiseVel = VelocityKt.getVelocity(LengthKt.getFeet(4));
	private static final Acceleration<Length> kDefaultAcceleration = AccelerationKt.getAcceleration(LengthKt.getFeet(8));
	// private final DriveTrain drive;
	private Velocity<NativeUnit> kCruiseVelTicks;
	private Acceleration<NativeUnit> kAccelTicks;
  private Command mCommand;
  private Length distance;
  private Velocity<Length> vel;
  private boolean reversed;
  private boolean commandStarted = false;

	/**
	 * Drive a distance (straight) forwards or backwards using a spliney boi.
	 * @param distance how far to move
	 * @param reversed if the path is reversed
	 */
	public DriveDistanceTheSecond(Length distance, boolean reversed) {
		this(distance, kCruiseVel, reversed);
	}

	public DriveDistanceTheSecond(Length distance, Velocity<Length> vel, boolean reversed) {
    this.distance = distance;
    this.vel = vel;
    this.reversed = reversed;
  }


	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
    List<Pose2d> waypoints = (!reversed) ? 
      Arrays.asList(
        new Pose2d(LengthKt.getInch(0), LengthKt.getInch(30), Rotation2dKt.getDegree(0)), 
        new Pose2d(distance, LengthKt.getInch(30), Rotation2dKt.getDegree(0))) 
      : Arrays.asList(
        new Pose2d(LengthKt.getInch(0), LengthKt.getInch(30), Rotation2dKt.getDegree(0)), 
        new Pose2d(distance.times(-1), LengthKt.getInch(30), Rotation2dKt.getDegree(0)));

    var trajectory = Trajectories.generateTrajectoryLowGear(waypoints, reversed);

    System.out.println("FIRST POSE: " + Util.toString(trajectory.getFirstState().getState().getPose()) + " LAST POSE: " + Util.toString(trajectory.getLastState().getState().getPose()));

    mCommand = DriveTrain.getInstance().followTrajectory(trajectory, TrajectoryTrackerMode.RAMSETE, true);

    mCommand.start();
    commandStarted = true;
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return mCommand.isCompleted() && commandStarted;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {}
}
