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

	
	public DriveMotorState(double leftVel, double leftAcc, double rightVel, double rightAcc, boolean brakeMode) {
		this.leftVel = leftVel;
		this.leftAcc = leftAcc;
		this.rightVel = rightVel;
		this.rightAcc = rightAcc;
		this.mBrakeMode = brakeMode;
		this.mAccelMode = true;
	}

	public DriveMotorState(double leftVel, double leftAcc, double rightVel, double rightAcc) {
		this.leftVel = leftVel;
		this.leftAcc = leftAcc;
		this.rightVel = rightVel;
		this.rightAcc = rightAcc;
		this.mBrakeMode = false;
		this.mAccelMode = true;
	}

	public DriveMotorState(double leftVel, double rightVel) {
		this.leftVel = leftVel;
		this.leftAcc = 0;
		this.rightVel = rightVel;
		this.rightAcc = 0;
		this.mBrakeMode = false;
		this.mAccelMode = true;
	}

	public DriveMotorState(double leftVel, double rightVel, boolean brakeMode) {
		this.leftVel = leftVel;
		this.leftAcc = 0;
		this.rightVel = rightVel;
		this.rightAcc = 0;
		this.mBrakeMode = brakeMode;
		this.mAccelMode = true;
	}


}
