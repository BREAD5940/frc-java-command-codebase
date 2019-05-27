//package frc.robot.commands.subsystems.drivetrain;
//
//import java.util.Arrays;
//
//import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
//import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
//import org.ghrobotics.lib.mathematics.units.Length;
//import org.ghrobotics.lib.mathematics.units.LengthKt;
//import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
//import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration;
//import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
//import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
//import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
//import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit;
//
//import org.team5940.pantry.exparimental.command.Command;
//import org.team5940.pantry.exparimental.command.SendableCommandBase;
//import frc.robot.commands.auto.Trajectories;
//import frc.robot.lib.motion.Util;
//import frc.robot.subsystems.DriveTrain;
//import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;
//
//public class DriveDistanceTheSecond extends SendableCommandBase{
//
//	private static Velocity<Length> kCruiseVel = VelocityKt.getVelocity(LengthKt.getFeet(4));
//	private static Acceleration<Length> kDefaultAcceleration = AccelerationKt.getAcceleration(LengthKt.getFeet(8));
//	// private final DriveTrain drive;
//	private Velocity<NativeUnit> kCruiseVelTicks;
//	private Acceleration<NativeUnit> kAccelTicks;
//	private Command mCommand;
//	private Length distance;
//	private Velocity<Length> vel;
//	private boolean reversed;
//	private boolean commandStarted = false;
//
//	/**
//	 * Drive a distance (straight) forwards or backwards using a spliney boi.
//	 * @param distance how far to move
//	 * @param reversed if the path is reversed
//	 */
//	public DriveDistanceTheSecond(Length distance, boolean reversed) {
//		this(distance, kCruiseVel, reversed);
//	}
//
//	public DriveDistanceTheSecond(Length distance, Velocity<Length> vel, boolean reversed) {
//		// this.distance = distance;
//		// this.vel = vel;
//		// this.reversed = reversed;
//		this.distance = distance;
//		this.vel = vel;
//		this.reversed = reversed;
//		addRequirements(DriveTrain.getInstance());
//	}
//
//	@Override
//	public void initialize() {
//
//		var currentPose = new Pose2d();
//		System.out.println("CURRENT POSE: " + currentPose.getTranslation().getX().getInch() + "," + currentPose.getTranslation().getY().getInch());
//		Translation2d offsetTrans;
//		if (!reversed) {
//			offsetTrans = new Translation2d(distance, currentPose.getRotation());
//		} else {
//			offsetTrans = new Translation2d(distance.times(-1), currentPose.getRotation());
//		}
//		var offsetPose = currentPose.plus(new Pose2d(offsetTrans, Rotation2dKt.getDegree(0)));
//
//		System.out.println("OFFSET POSE: " + offsetPose.getTranslation().getX().getInch() + "," + offsetPose.getTranslation().getY().getInch());
//
//		var waypoints = Arrays.asList(
//				currentPose, offsetPose);
//
//		// List<Pose2d> waypoints = (!reversed) ? Arrays.asList(
//		// 	new Pose2d(LengthKt.getInch(0), LengthKt.getInch(30), Rotation2dKt.getDegree(0)),
//		// 	new Pose2d(distance, LengthKt.getInch(30), Rotation2dKt.getDegree(0)))
//		// 	: Arrays.asList(
//		// 			new Pose2d(LengthKt.getInch(0), LengthKt.getInch(30), Rotation2dKt.getDegree(0)),
//		// 			new Pose2d(distance.times(-1), LengthKt.getInch(30), Rotation2dKt.getDegree(0)));
//
//		var trajectory = Trajectories.generateTrajectoryHighGear(waypoints, reversed);
//
//		System.out.println("FIRST POSE: " + Util.toString(trajectory.getFirstState().getState().getPose()) + " LAST POSE: " + Util.toString(trajectory.getLastState().getState().getPose()));
//
//		clearRequirements();
//
//		mCommand = DriveTrain.getInstance().followTrajectory(trajectory, TrajectoryTrackerMode.RAMSETE, true);
//
//		// addSequential(mCommand);
//		clearRequirements();
//		mCommand.schedule();
//
//		// mCommand.schedule();
//		commandStarted = true;
//
//	}
//
//	// 	// Called just before this Command runs the first time
//	// 	@Override
//	// 	public void initialize() {
//
//	// 	}
//
//	// 	// Called repeatedly when this Command is scheduled to run
//	// 	@Override
//	// 	public void execute() {}
//
//	// Make this return true when this Command no longer needs to run execute()
//	@Override
//	public boolean isFinished() {
//		return mCommand.isCompleted() && commandStarted;
//	}
//
//	// 	// Called once after isFinished returns true
//	// 	@Override
//	// 	public void end(boolean interrupted) {}
//
//	// 	// Called when another command which requires one or more of the same
//	// 	// subsystems is scheduled to run
//	// 	@Override
//	// 	protected void interrupted() {}
//}
