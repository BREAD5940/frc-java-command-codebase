package frc.robot.commands.subsystems.drivetrain;

import java.util.Arrays;
import java.util.function.Supplier;

import org.ghrobotics.lib.debug.LiveDashboard;
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TrajectorySamplePoint;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit;
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerOutput;

import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Robot;
import frc.robot.commands.auto.Trajectories;
import frc.robot.lib.AutoCommand;
import frc.robot.lib.Logger;
import frc.robot.subsystems.DriveTrain;

// @SuppressWarnings({"WeakerAccess", "unused"})
public class DriveDistanceTheThird extends AutoCommand {
	private TrajectoryTracker trajectoryTracker;
	private Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySource;
	private DriveTrain driveBase;
	private boolean reset;
	private TrajectoryTrackerOutput output;
	// TODO make sure that this fabled namespace collision doesn't happen on
	// Shuffleboard
	Length mDesiredLeft;
	Length mDesiredRight;
	double mCurrentLeft;
	double mCurrentRight;

	Notifier mUpdateNotifier;

	private static Velocity<Length> kCruiseVel = VelocityKt.getVelocity(LengthKt.getFeet(2));
	private static Acceleration<Length> kDefaultAcceleration = AccelerationKt.getAcceleration(LengthKt.getFeet(8));
	// private final DriveTrain drive;
	private Velocity<NativeUnit> kCruiseVelTicks;
	private Acceleration<NativeUnit> kAccelTicks;
	// private Command mCommand;
	private Length distance;
	private Velocity<Length> vel;
	private boolean reversed;
	private boolean commandStarted = false;

	/**
	 * Drive a distance (straight) forwards or backwards using a spliney boi.
	 * @param distance how far to move
	 * @param reversed if the path is reversed
	 */
	public DriveDistanceTheThird(Length distance, boolean reversed) {
		this(distance, kCruiseVel, reversed);
	}

	public DriveDistanceTheThird(Length distance, Velocity<Length> vel, boolean reversed) {
		// this.distance = distance;
		// this.vel = vel;
		// this.reversed = reversed;
		driveBase = DriveTrain.getInstance();
		this.trajectoryTracker = driveBase.getTrajectoryTracker();
		this.distance = distance;
		this.vel = vel;
		this.reversed = reversed;
		requires(DriveTrain.getInstance());
	}

	@Override
	protected void initialize() {

		var currentPose = DriveTrain.getInstance().getLocalization().getRobotPosition();
		System.out.println("CURRENT POSE: " + currentPose.getTranslation().getX().getInch() + "," + currentPose.getTranslation().getY().getInch() + "," + currentPose.getRotation().getDegree());
		Translation2d offsetTrans;
		if(!reversed) {
			offsetTrans = new Translation2d(distance, currentPose.getRotation().plus(Rotation2dKt.getDegree(-0)));
			// offsetTrans = new Translation2d(distance.times(currentPose.getRotation().plus(Rotation2dKt.getDegree(90)).getCos()), distance.times(currentPose.getRotation().plus(Rotation2dKt.getDegree(90)).getSin()));
		} else {
			offsetTrans = new Translation2d(distance, currentPose.getRotation().plus(Rotation2dKt.getDegree(-0)));
		}
		
		System.out.println("Calculated offset is " + offsetTrans.getX().getInch() + "," + offsetTrans.getY().getInch());

		var offsetPose = currentPose.plus(new Pose2d(offsetTrans, Rotation2dKt.getDegree(0)));

		System.out.println("OFFSET POSE: " + offsetPose.getTranslation().getX().getInch() + "," + offsetPose.getTranslation().getY().getInch() + "," + offsetPose.getRotation().getDegree());


		var waypoints = Arrays.asList(
			currentPose, offsetPose
		);

		// List<Pose2d> waypoints = (!reversed) ? Arrays.asList(
		// 	new Pose2d(LengthKt.getInch(0), LengthKt.getInch(30), Rotation2dKt.getDegree(0)),
		// 	new Pose2d(distance, LengthKt.getInch(30), Rotation2dKt.getDegree(0)))
		// 	: Arrays.asList(
		// 			new Pose2d(LengthKt.getInch(0), LengthKt.getInch(30), Rotation2dKt.getDegree(0)),
		// 			new Pose2d(distance.times(-1), LengthKt.getInch(30), Rotation2dKt.getDegree(0)));

	var trajectory = Trajectories.generateTrajectoryLowGear(waypoints, reversed);		




		trajectorySource = () -> (trajectory);

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

		Logger.log("desired linear, real linear");

		LiveDashboard.INSTANCE.setFollowingPath(true);

		mUpdateNotifier = new Notifier(() -> {
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
