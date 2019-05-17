// package frc.robot.commands.auto.actions;

// import org.team5940.pantry.experimental.command.SendableCommandBase;
// import frc.robot.Robot;
// import frc.robot.RobotConfig;
// import frc.robot.lib.Logger;
// import frc.robot.lib.TerriblePID;
// import frc.robot.lib.motion.Util;

// /**
//  * Follow a vision target tracked by a limelight
//  * It's pretty terrible right now
//  * 
//  * @author Matthew Morley
//  */
// public class FollowVisionTarget extends SendableCommandBase {
// 	double timeout,
// 			targetSpeed,
// 			targetPercentOfFrame,
// 			angleDeltaX,
// 			angleDeltaY,
// 			forwardSpeed,
// 			turnSpeed,
// 			leftSpeedRaw,
// 			rightSpeedRaw;
// 	boolean followRange = false;

// 	double targetSizeSetpoint = 6;

// 	boolean noCurrentTarget = false;

// 	boolean tooClose = false;

// 	boolean hadTarget = false;
// 	double lastKnownYaw;

// 	private TerriblePID forwardPID = new TerriblePID(
// 			RobotConfig.auto.followVisionTarget.forward.kp,
// 			RobotConfig.auto.drive_auto_forward_velocity_max);
// 	private TerriblePID turnPID = new TerriblePID(
// 			RobotConfig.auto.followVisionTarget.turn.kp,
// 			RobotConfig.auto.followVisionTarget.turn.max_turn_speed);

// 	/**
// 	 * Follow a limelight vision target. Move toward the target at the set speed 
// 	 * and timeout after a set time. Tracks only angle, not range!
// 	 * @param speed
// 	 * @param timeout
// 	 */
// 	public FollowVisionTarget(double speed, double timeout) {
// 		this.timeout = timeout;
// 		this.targetSpeed = speed;
// 		requires(Robot.drivetrain);
// 	}

// 	/**
// 	 * Follow a limelight vision target. Track both range and angle
// 	 * of the target to the limelight
// 	 * @param speed to track forwards at
// 	 * @param targetPercentOfFrame percent of frame taken up by target
// 	 * @param timeout
// 	 */
// 	public FollowVisionTarget(double speed, double targetPercentOfFrame, double timeout) {
// 		this.timeout = timeout;
// 		this.targetSpeed = speed;
// 		this.targetPercentOfFrame = targetPercentOfFrame;
// 		this.followRange = true;
// 		requires(Robot.drivetrain);
// 	}

// 	// Called just before this Command runs the first time
// 	@Override
// 	public void initialize() {
// 		setTimeout(timeout);
// 		turnPID.setSetpoint(0);
// 		forwardPID.setMaxOutput(targetSpeed);

// 		if (followRange) {
// 			forwardPID.setSetpoint(targetPercentOfFrame);
// 			forwardPID.setKpGain(RobotConfig.auto.followVisionTarget.forward.kp_rangeMode);
// 		} else {
// 			forwardPID.setSetpoint(targetSpeed);
// 		}

// 		angleDeltaX = Robot.limelight.getDx();
// 		targetPercentOfFrame = Robot.limelight.getTargetArea();

// 		Logger.log("Command init");

// 	}

// 	// Called repeatedly when this Command is scheduled to run
// 	@Override
// 	public void execute() {
// 		if (Robot.limelight.getData()[0] != 0) {
// 			noCurrentTarget = false;
// 			hadTarget = true;

// 			double[] data = Robot.limelight.getData();
// 			double limelightData = data[1];
// 			double sizeData = data[3];

// 			lastKnownYaw = limelightData;
// 			// turnSpeed = ;
// 			turnSpeed = Util.limit(limelightData * (1 / 10), -0.5, 0.5);

// 			Logger.log("Turn speed: " + turnSpeed);

// 			forwardSpeed = 0;

// 			double distanceRatio = targetSizeSetpoint - sizeData;

// 			double forwardSpeed = distanceRatio * 0.2;

// 			if (sizeData > targetSizeSetpoint) {
// 				tooClose = true;
// 				Logger.log("Too close");
// 			}

// 			if (forwardSpeed > 0.5) {
// 				forwardSpeed = 0.5;
// 			}
// 			if (forwardSpeed < -0.5) {
// 				forwardSpeed = -0.5;
// 			}

// 			// System.out.println("forward speed: " + forwardSpeed + " Turn speed: " + turnSpeed);

// 			// Robot.drivetrain.setSpeeds(leftSpeedRaw, rightSpeedRaw);
// 			// Robot.drivetrain.setPowers(forwardSpeed + turnSpeed, forwardSpeed - turnSpeed);

// 			// Robot.drivetrain.setFeetPerSecond(0.5,0.5);

// 		} else {
// 			noCurrentTarget = true;
// 		}
// 	}

// 	// Make this return true when this Command no longer needs to run execute()
// 	@Override
// 	public boolean isFinished() {
// 		double datainYaw = Robot.limelight.getData()[1];
// 		double datainSize = Robot.limelight.getData()[3];

// 		Logger.log("Am I done? " + (isTimedOut() || (hadTarget && noCurrentTarget) || (((datainSize > targetSizeSetpoint)) || (datainYaw < 0.5))));

// 		Logger.log("Do I have no target but I used to?? " + ((hadTarget && noCurrentTarget)));
// 		Logger.log("Am I within yaw tolerence?" + (lastKnownYaw < 0.5 && lastKnownYaw != 0));
// 		Logger.log("Am I too close?" + (datainSize > targetSizeSetpoint));
// 		Logger.log("is the yaw within tolerence? " + (datainYaw < 0.5));

// 		return (isTimedOut() || (hadTarget && noCurrentTarget) || (((datainSize > targetSizeSetpoint)) && (datainYaw < 0.5)));
// 	}

// 	// Called once after isFinished returns true
// 	@Override
// 	protected void end() {}

// 	// Called when another command which requires one or more of the same
// 	// subsystems is scheduled to run
// 	@Override
// 	protected void interrupted() {}
// }
