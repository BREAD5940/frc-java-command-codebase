package frc.robot.commands.subsystems.drivetrain;

import java.util.function.Supplier;

import org.ghrobotics.lib.debug.LiveDashboard;
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TrajectorySamplePoint;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerOutput;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.commands.auto.Trajectories;
import frc.robot.lib.AutoCommand;
import frc.robot.lib.Logger;
import frc.robot.subsystems.DriveTrain;

// @SuppressWarnings({"WeakerAccess", "unused"})
public class TrajectoryTrackerCommand extends AutoCommand {
	private TrajectoryTracker trajectoryTracker;
	private Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySource;
	private DriveTrain driveBase;
	private boolean reset;
	private TrajectoryTrackerOutput output;
	// TODO make sure that this fabled namespace collision doesn't happen on Shuffleboard 
	Length mDesiredLeft;
	Length mDesiredRight;
	double mCurrentLeft;
	double mCurrentRight;

	// private NetworkTableEntry refVelEntry = Shuffleboard.getTab("Auto").getLayout("List", "Pathing info").add("Reference Velocity", 0).getEntry();
	// private NetworkTableEntry currentVelEntry = Shuffleboard.getTab("Auto").getLayout("List", "Pathing info").add("Current Velocity", 0).getEntry();

	Notifier mUpdateNotifier;

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

		// Logger.log("desired linear, real linear");

		LiveDashboard.INSTANCE.setFollowingPath(true);

		mUpdateNotifier = new Notifier(() -> {
			output = trajectoryTracker.nextState(driveBase.getRobotPosition(), TimeUnitsKt.getSecond(Timer.getFPGATimestamp()));

			TrajectorySamplePoint<TimedEntry<Pose2dWithCurvature>> referencePoint = trajectoryTracker.getReferencePoint();
			if (referencePoint != null) {
				Pose2d referencePose = referencePoint.getState().getState().getPose();

				LiveDashboard.INSTANCE.setPathX(referencePose.getTranslation().getX().getFeet());
				LiveDashboard.INSTANCE.setPathY(referencePose.getTranslation().getY().getFeet());
				LiveDashboard.INSTANCE.setPathHeading(referencePose.getRotation().getRadian());

				// Shuffleboard.getTab("Auto").getLayout("Pathing", BuiltInLayouts.kList).

				// refVelEntry.setDouble(referencePoint.getState().getVelocity().getType$FalconLibrary().getFeet());
				// currentVelEntry.setDouble(driveBase.getLeft().getFeetPerSecond());

			}
			// Logger.log("Linear: " + output.getLinearVelocity().getValue() + " Angular: " + output.getAngularVelocity().getValue() );
			driveBase.setOutput(output);
		});
		mUpdateNotifier.startPeriodic(0.01);
	}

	@Override
	protected void execute() {

		// long now = System.currentTimeMillis();

		// output = trajectoryTracker.nextState(driveBase.getRobotPosition(), TimeUnitsKt.getMillisecond(System.currentTimeMillis()));

		// TrajectorySamplePoint<TimedEntry<Pose2dWithCurvature>> referencePoint = trajectoryTracker.getReferencePoint();
		// if (referencePoint != null) {
		// 	Pose2d referencePose = referencePoint.getState().getState().getPose();

		// 	LiveDashboard.INSTANCE.setPathX(referencePose.getTranslation().getX().getFeet());
		// 	LiveDashboard.INSTANCE.setPathY(referencePose.getTranslation().getY().getFeet());
		// 	LiveDashboard.INSTANCE.setPathHeading(referencePose.getRotation().getRadian());
		// }
		// // Logger.log("Linear: " + output.getLinearVelocity().getValue() + " Angular: " + output.getAngularVelocity().getValue() );
		// driveBase.setOutput(output);

		// DriveDynamics desired_vel = driveBase.getDifferentialDrive().solveInverseDynamics(output.getDifferentialDriveVelocity(), output.getDifferentialDriveAcceleration());

		// mDesiredLeft = LengthKt.getFeet(desired_vel.getVoltage().getLeft() * driveBase.getDifferentialDrive().getWheelRadius());
		// mDesiredRight = LengthKt.getFeet(desired_vel.getVoltage().getRight() * driveBase.getDifferentialDrive().getWheelRadius());
		// mCurrentLeft = driveBase.getLeft().getFeetPerSecond();
		// mCurrentRight = driveBase.getRight().getFeetPerSecond();

		// SmartDashboard.putNumberArray("trackerInfo", new double[]{mDesiredLeft.getFeet(), mDesiredRight.getFeet(), mCurrentLeft, mCurrentRight});
	}

	@Override
	protected void end() {
		mUpdateNotifier.stop();
		driveBase.stop();
		LiveDashboard.INSTANCE.setFollowingPath(false);
	}

	@Override
	protected boolean isFinished() {
		return trajectoryTracker.isFinished();
	}

	public TimedTrajectory<Pose2dWithCurvature> getTrajectory() {
		return this.trajectorySource.get();
	}

}
