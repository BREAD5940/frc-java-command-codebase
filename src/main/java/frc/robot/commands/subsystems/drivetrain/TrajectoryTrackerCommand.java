package frc.robot.commands.subsystems.drivetrain;

import java.util.function.Supplier;

import org.ghrobotics.lib.debug.LiveDashboard;
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TrajectorySamplePoint;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerOutput;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.commands.auto.Trajectories;
import frc.robot.lib.Logger;
import frc.robot.subsystems.DriveTrain;

// @SuppressWarnings({"WeakerAccess", "unused"})
public class TrajectoryTrackerCommand extends Command {
	private TrajectoryTracker trajectoryTracker;
	private Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySource;
	private DriveTrain driveBase;
	private boolean reset;
	private TrajectoryTrackerOutput output;
	// private DifferentialDrive mModel;

	public TrajectoryTrackerCommand(DriveTrain driveBase, Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySource) {
		this(driveBase, trajectorySource, false);
	}

	public TrajectoryTrackerCommand(DriveTrain driveBase, Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySource, boolean reset) {
		this(driveBase, Robot.drivetrain.getTrajectoryTracker(), trajectorySource, reset);
	}

	public TrajectoryTrackerCommand(DriveTrain driveBase, TrajectoryTracker trajectoryTracker, Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySource, boolean reset) {
		requires(driveBase);
		this.driveBase = driveBase;
		this.trajectoryTracker = trajectoryTracker;
		this.trajectorySource = trajectorySource;
		this.reset = reset;
		// this.mModel = driveBase.getDifferentialDrive();
	}

	@Override
	protected void initialize() {
		LiveDashboard.INSTANCE.setFollowingPath(false);

		if (trajectorySource == null) {
			Logger.log("Sadly the trajectories are not generated. the person responsible for the trajectories has been sacked.");
			Trajectories.generateAllTrajectories();
		}

		Logger.log("get: " + trajectorySource.get().getFirstState().getState().getCurvature().toString());

		trajectoryTracker.reset(this.trajectorySource.get());

		Logger.log("first pose: " + trajectorySource.get().getFirstState().getState().getPose().getTranslation().getX().getFeet());

		if (reset == true) {
			Robot.drivetrain.getLocalization().reset(trajectorySource.get().getFirstState().getState().getPose());
		}

		LiveDashboard.INSTANCE.setFollowingPath(true);
	}

	@Override
	protected void execute() {

		// long now = System.currentTimeMillis();

		output = trajectoryTracker.nextState(driveBase.getRobotPosition(), TimeUnitsKt.getMillisecond(System.currentTimeMillis()));

		TrajectorySamplePoint<TimedEntry<Pose2dWithCurvature>> referencePoint = trajectoryTracker.getReferencePoint();
		if (referencePoint != null) {
			Pose2d referencePose = referencePoint.getState().getState().getPose();

			LiveDashboard.INSTANCE.setPathX(referencePose.getTranslation().getX().getFeet());
			LiveDashboard.INSTANCE.setPathY(referencePose.getTranslation().getY().getFeet());
			LiveDashboard.INSTANCE.setPathHeading(referencePose.getRotation().getRadian());
		}
		// Logger.log("Linear: " + output.getLinearVelocity().getValue() + " Angular: " + output.getAngularVelocity().getValue() );
		driveBase.setOutput(output);

		// long elapsed = System.currentTimeMillis() - now;
		// System.out.println("Took " + elapsed + "ms");
	}

	@Override
	protected void end() {
		driveBase.stop();
		LiveDashboard.INSTANCE.setFollowingPath(false);
	}

	@Override
	protected boolean isFinished() {
		return trajectoryTracker.isFinished();
	}
}
