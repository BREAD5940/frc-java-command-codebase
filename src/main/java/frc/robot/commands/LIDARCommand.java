package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.RobotConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class LIDARCommand extends Command{

    public LIDARCommand() {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.lidarSubsystem);
    }
    // Called just before this Command runs the first time
    protected void initialize() {
    	try {
    	Robot.lidarSubsystem.initLIDAR(new DigitalInput(RobotConfig.lidar.port));
    	}catch(Exception e) {
    		
    	}
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	SmartDashboard.putNumber("LIDAR Distance",Robot.lidarSubsystem.getDistanceIn());
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}