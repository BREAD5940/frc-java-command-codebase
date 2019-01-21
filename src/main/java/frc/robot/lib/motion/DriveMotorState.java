package frc.robot.lib.motion;

/**
 * The DriveMotorState class represents a commanded drivetrain state. It stores compoenent
 * velocities, accelerations, motor modes and acceleration modes.
 */
public class DriveMotorState {
	public double leftVel, leftAcc;
	public double rightVel, rightAcc;
	public static DriveMotorState NEUTRAL = new DriveMotorState(0, 0, 0, 0);
	public static DriveMotorState BRAKE = new DriveMotorState(0, 0, 0, 0, true);
	public boolean mBrakeMode;
	public boolean mAccelMode;

	/**
	 * This drivemotorstate is set using left and right velocities and accelerations in addition to a brakeMode bool
	 * @param leftVel
	 * @param leftAcc
	 * @param rightVel
	 * @param rightAcc
	 * @param brakeMode
	 */	
	public DriveMotorState(double leftVel, double leftAcc, double rightVel, double rightAcc, boolean brakeMode) {
		this.leftVel = leftVel;
		this.leftAcc = leftAcc;
		this.rightVel = rightVel;
		this.rightAcc = rightAcc;
		this.mBrakeMode = brakeMode;
		this.mAccelMode = true;
	}

	/**
	 * This drivemotorstate is not in brake mode and contains left and right velocities and accelerations
	 * @param leftVel
	 * @param leftAcc
	 * @param rightVel
	 * @param rightAcc
	 */
	public DriveMotorState(double leftVel, double leftAcc, double rightVel, double rightAcc) {
		this.leftVel = leftVel;
		this.leftAcc = leftAcc;
		this.rightVel = rightVel;
		this.rightAcc = rightAcc;
		this.mBrakeMode = false;
		this.mAccelMode = true;
	}

	/**
	 * This drivemotorstate contains only left and right vels, and brake mode and acceleration control
	 * is false.
	 * @param leftVel
	 * @param rightVel
	 */
	public DriveMotorState(double leftVel, double rightVel) {
		this.leftVel = leftVel;
		this.leftAcc = 0;
		this.rightVel = rightVel;
		this.rightAcc = 0;
		this.mBrakeMode = false;
		this.mAccelMode = false;
	}

	/**
	 * Create a new drive motor state object with left and right velocities. In
	 * this mode acceleration is not controlled and brake mode is user set.
	 * @param leftVel
	 * @param rightVel
	 * @param brakeMode
	 */
	public DriveMotorState(double leftVel, double rightVel, boolean brakeMode) {
		this.leftVel = leftVel;
		this.leftAcc = 0;
		this.rightVel = rightVel;
		this.rightAcc = 0;
		this.mBrakeMode = brakeMode;
		this.mAccelMode = false;
	}

	@Override
	public String toString() {
		return "DriveMotorState, leftVel, " + leftVel +", leftAcc, " + leftAcc +", rightVel, " + rightVel +
						", rightAcc, " + rightAcc +", mBrakeMode, " + mBrakeMode + ", mAccelMode, " + mAccelMode;
	}

}
