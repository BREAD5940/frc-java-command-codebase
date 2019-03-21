import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Twist2d;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.junit.jupiter.api.Test;

import com.team254.lib.physics.DifferentialDrive;
import com.team254.lib.physics.DifferentialDrive.ChassisState;
import com.team254.lib.physics.DifferentialDrive.WheelState;

import frc.robot.Constants;
import frc.robot.lib.motion.Util;

public class DifferentialDriveTest {

	static Pose2d robotPose = new Pose2d();
	static DifferentialDrive diffDrive = Constants.kLowGearDifferentialDrive;

	@Test
	public void testTurnKinematics() {
		print("error, degPerSec");
		for (int i = 0; i < 25; i++) {
			parseError(i);
		}
	}

	void print(Object o) {
		System.out.println(o.toString());
	}

	public void parseError(double error) {
		// System.out.println("error: " + error);
		final double maxYawSpeed = 2; //feet per sec

		// var error = 1;
		var turnPower = error * 0.1;
		turnPower = Util.limit(turnPower, maxYawSpeed);

		ChassisState chassisMotion = new ChassisState(0, Util.toMeters(turnPower));
		var chassisVelocity = diffDrive.solveInverseKinematics(chassisMotion);
		// System.out.println(chassisVelocity.getLeft() + "," + chassisVelocity.getRight());

		var wheelVelocities = diffDrive.solveInverseKinematics(chassisMotion);
		var feedForwardVoltages = diffDrive.getVoltagesFromkV(wheelVelocities);

		print(error + "\t" + wheelVelocities.getLeft() * 360 / Math.PI);

		// System.out.println("voltages: " + feedForwardVoltages.getLeft() / 12 + "," + feedForwardVoltages.getRight() / 12);
		// System.out.println("rotations per sec: " + wheelVelocities.getLeft() * 1 / Math.PI + "," + wheelVelocities.getRight() * 1 / Math.PI);
		// System.out.println("degrees per sec: " + wheelVelocities.getLeft() * 360 / Math.PI + "," + wheelVelocities.getRight() * 360 / Math.PI);

		// System.out.println("=======================================================================================");
	}

	public void updatePose(WheelState states) {

		var forwardKinematics = diffDrive.solveForwardKinematics(states);

		robotPose = robotPose.plus(
				new Twist2d(
						forwardKinematics.getLinear(),
						0.0,
						Rotation2dKt.getRadian(forwardKinematics.getAngular() * 1)).getAsPose());

	}

}
