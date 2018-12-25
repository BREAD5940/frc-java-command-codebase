package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.lib.EncoderLib;

public class SetWrist extends Command {
    boolean isInstant;
    double target_angle;
    double position_tolerence = EncoderLib.degreesToRaw(RobotConfig.wrist_position_tolerence,RobotConfig.POSITION_PULSES_PER_ROTATION);
    double velocity_tolerence = EncoderLib.degreesToRaw(RobotConfig.wrist_velocity_tolerence, RobotConfig.POSITION_PULSES_PER_ROTATION);   // TODO verify this behavior, maybe omit for now?

    /**
     * This constructor takes a target angle in degrees and the boolean isInstant.
     * If you want thsi command to return instantly, use the one argument constructor 
     * unless you want to be verbose. If isInstant is true, this command will only
     * return if both position and angular velocity are within thresholds.
     * @param target_angle in degrees from zero
     * @param isInstant boolean
     */
    public SetWrist(double target_angle, boolean isInstant) {
        this.isInstant = isInstant;
        this.target_angle = target_angle;
        requires(Robot.wrist); // reserve the wrist subsystem
    }

    /**
     * Boring version of the set wrist constructor. Defaults to instant return without waiting for angle.
     * If you want to wait for the wrist to reach the target angle, set the second argument to false.
     * @param target_angle in degrees from zero
     */
    public SetWrist(double target_angle) {
        this.isInstant = true; // TODO review if the default behavior of the wrist (to not wait for angle before returning) is desirable
        this.target_angle = target_angle;
        requires(Robot.wrist); // reserve the wrist subsystem
    }

    // public static final drivetrain drivetrain  = new drivetrain();

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.wrist.setAngle(target_angle);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() { 
        System.out.println("Current wrist angle: " + Robot.wrist.getAngle() + " Target wrist angle: " + target_angle);
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        if (isInstant || //if command has to run instantly, return true - otherwise check all the conditions
            (( Math.abs(Robot.wrist.getAngle() - target_angle ) < position_tolerence ) 
            && (Math.abs(Robot.wrist.getAngularVelocity()) < velocity_tolerence))) {
                return true;
            }
        else { 
            return false;
        }
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
    }
}