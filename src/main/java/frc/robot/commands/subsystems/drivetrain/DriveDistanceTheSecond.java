package frc.robot.commands.subsystems.drivetrain;

import java.util.Arrays;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.auto.Trajectories;
import frc.robot.lib.AutoCommand;
import frc.robot.subsystems.DriveTrain;

public class DriveDistanceTheSecond extends Command {

	private static final Velocity<Length> kCruiseVel = VelocityKt.getVelocity(LengthKt.getFeet(4));
	private static final Acceleration<Length> kDefaultAcceleration = AccelerationKt.getAcceleration(LengthKt.getFeet(8));
	private final DriveTrain drive;
	private Velocity<NativeUnit> kCruiseVelTicks;
	private Acceleration<NativeUnit> kAccelTicks;
	private AutoCommand mCommand;

	/**
	 * Drive a distance (straight) forwards or backwards using a spliney boi.
	 * @param distance how far to move
	 * @param reversed if the path is reversed
	 */
	public DriveDistanceTheSecond(Length distance, boolean reversed) {
		this(distance, kCruiseVel, reversed);
	}

	public DriveDistanceTheSecond(Length distance, Velocity<Length> vel, boolean reversed) {
		this(distance, vel, VelocityKt.getVelocity(LengthKt.getFeet(0)), VelocityKt.getVelocity(LengthKt.getFeet(0)), false);
	}

	public DriveDistanceTheSecond(Length distance, Velocity<Length> vel, Velocity<Length> startV, Velocity<Length> endV, boolean reversed) {
		this.drive = DriveTrain.getInstance();
		var mCurrentPose = drive.getLocalization().getRobotPosition();
		var mDelta = new Pose2d(new Translation2d(distance, mCurrentPose.getRotation()), mCurrentPose.getRotation());
		this.mCommand = drive.followTrajectory(Trajectories.generateTrajectoryLowGear(Arrays.asList(mCurrentPose, mDelta), reversed));
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		mCommand.start();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return mCommand.done();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {}
}
