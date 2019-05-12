package frc.robot.commands.subsystems.drivetrain;

import java.util.function.Supplier;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.SILengthConstants;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;

import com.team254.lib.physics.DifferentialDrive.ChassisState;

import org.team5940.pantry.exparimental.command.SendableCommandBase;
import frc.robot.lib.motion.Util;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;

public class DriveDistanceToVisionTarget extends SendableCommandBase {

	private final Length finalDistance;
	// private final Velocity<Length> 

	private Supplier<Length> targetDistance;
	private Supplier<Rotation2d> targetYaw;
	boolean isWithinDelta = false;

	private final double ticksToMax = 3;
	double lastFwdCmd = 0;

	private Velocity<Length> velocity;

	public DriveDistanceToVisionTarget(Length endDistance, Velocity<Length> speed) {

		this.finalDistance = endDistance;

		this.velocity = speed;

		targetDistance = () -> {
			// return LimeLight.getInstance().getPose(0).getTranslation().getX();
			return LimeLight.getInstance().estimateDistanceFromAngle();
		};
		targetYaw = () -> {
			return LimeLight.getInstance().getDx();
		};

	}

	@Override
	public void execute() {
		var distanceToGo = targetDistance.get().minus(finalDistance);

		final double fwdKp = 6;
		final double yawKp = 0.2; // TODO tuneme
		final double maxYawSpeed = 4; //feet per sec

		var fwdPower = Util.limit(distanceToGo.getFeet() * fwdKp, velocity.getValue() / SILengthConstants.kFeetToMeter);
		fwdPower = Util.limit(fwdPower, lastFwdCmd - (1 / ticksToMax), lastFwdCmd + (1 / ticksToMax));

		var steer_cmd = targetYaw.get().getDegree() * yawKp;
		steer_cmd = Util.limit(steer_cmd, maxYawSpeed);

		lastFwdCmd = fwdPower;

		var commandedChassisState = new ChassisState(
				Util.toMeters(fwdPower),
				Util.toMeters(steer_cmd) // ah shit i can't believe you've done this
		);

		DriveTrain.getInstance().setOutputFromKinematics(commandedChassisState);

		isWithinDelta = Math.abs(distanceToGo.getFeet()) < (2f / 12f) && (Math.abs(targetYaw.get().getDegree()) < 3);
	}

	@Override
	public boolean isFinished() {
		return isWithinDelta;
	}

}
