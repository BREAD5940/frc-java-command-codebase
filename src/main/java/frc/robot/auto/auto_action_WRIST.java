package frc.robot.auto;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.lib.EncoderLib;

public class auto_action_WRIST extends Command {
    boolean wait_for_wrist;
    double target_angle;
    boolean am_i_done = false;
    double position_tolerence = EncoderLib.degreesToRaw(RobotConfig.wrist_position_tolerence,RobotConfig.POSITION_PULSES_PER_ROTATION);
    double velocity_tolerence = EncoderLib.degreesToRaw(RobotConfig.wrist_velocity_tolerence, RobotConfig.POSITION_PULSES_PER_ROTATION);   // TODO verify this behavior, maybe omit for now?

    public auto_action_WRIST(double target_angle, boolean wait_for_wrist) {
    this.wait_for_wrist = wait_for_wrist;
    this.target_angle = target_angle;
    requires(Robot.wrist); // reserve the intake subsystem, TODO make sure this doesnt break anything
    // this.runtime = runtime;
    }

    // public static final drivetrain drivetrain  = new drivetrain();

    // Called just before this Command runs the first time
    @Override
    protected void initialize() { // TODO verify that I can just set the setpoint and walk away
        Robot.wrist.setAngle(target_angle);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() { 
        // Check exit condition of being within target angle and velocity
        if (( Math.abs(Robot.wrist.getAngle() - target_angle ) < position_tolerence ) 
            && (Math.abs(Robot.wrist.getAngularVelocity()) < velocity_tolerence)) {
                am_i_done = true;
            }
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        if (wait_for_wrist != true) { return true; }
        else { return am_i_done; }
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
