package frc.robot.auto.actions;

import java.io.File;

import frc.robot.Robot;
import frc.robot.RobotConfig;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;

import frc.robot.subsystems.DriveTrain.Gear;

public class auto_DriveTrajectoryPathfinder extends Command {
  private Trajectory m_leftTrajectory, m_rightTrajectory, m_sourceTrajectory;
  private DistanceFollower m_leftFollower, m_rightFollower;
  private double m_leftOutput, m_rightOutput, m_turn, m_angularError;
  private TankModifier m_modifier;

  public auto_DriveTrajectoryPathfinder(Trajectory traj) {
    m_sourceTrajectory = traj;
    m_modifier = new TankModifier(m_sourceTrajectory);
    m_modifier.modify(RobotConfig.driveTrain.drivetrain_width);    
    m_leftTrajectory = m_modifier.getLeftTrajectory();
    m_rightTrajectory = m_modifier.getRightTrajectory();
    requires(Robot.drivetrain);
  }

  public auto_DriveTrajectoryPathfinder(String file) {
    File traj = new File("/home/lvuser/deploy/paths/" + file + ".pf1.traj");
    SmartDashboard.putBoolean("Source exists", true);
    m_sourceTrajectory = Pathfinder.readFromFile(traj);
    File leftTraj = new File("/home/deploy/lvuser/paths/" + file + ".left.pf1.traj");
    SmartDashboard.putBoolean("Left exists", true);
    m_leftTrajectory = Pathfinder.readFromFile(leftTraj);
    File rightTraj = new File("/home/deploy/lvuser/paths/" + file + ".right.pf1.traj");
    SmartDashboard.putBoolean("Right exists", true);
    m_rightTrajectory = Pathfinder.readFromFile(rightTraj);
    requires(Robot.drivetrain);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    // if(RobotConfig.auto.default_auto_gear == "low"){
      double left_kp = RobotConfig.driveTrain.left_talons.velocity_kp_low;
      double left_ki = 0;//RobotConfig.driveTrain.left_talons.velocity_ki_low;
      double left_kd = RobotConfig.driveTrain.left_talons.velocity_kd_low;
      double left_kv = RobotConfig.driveTrain.left_talons.velocity_kv_low;
      double left_ka = 0;//RobotConfig.driveTrain.left_talons.velocity_ki_low;

      double right_kp = RobotConfig.driveTrain.left_talons.velocity_kp_low;
      double right_ki = 0;//RobotConfig.driveTrain.left_talons.velocity_ki_low;
      double right_kd = RobotConfig.driveTrain.left_talons.velocity_kd_low;
      double right_kv = RobotConfig.driveTrain.left_talons.velocity_kv_low;
      double right_ka = 0;//RobotConfig.driveTrain.left_talons.velocity_ka_low;
    // }

    // Modify the variables if the gear is high, yes this is a bit of a hack but 
    if ( RobotConfig.auto.auto_gear == Gear.HIGH ){
      left_kp = RobotConfig.driveTrain.left_talons.velocity_kp_high;
      left_ki = 0;//RobotConfig.driveTrain.left_talons.velocity_ki_high;
      left_kd = RobotConfig.driveTrain.left_talons.velocity_kd_high;
      left_kv = RobotConfig.driveTrain.left_talons.velocity_kv_high;
      left_ka = 0;//RobotConfig.driveTrain.left_talons.velocity_ki_high;

      right_kp = RobotConfig.driveTrain.left_talons.velocity_kp_high;
      right_ki = 0;//RobotConfig.driveTrain.left_talons.velocity_ki_high;
      right_kd = RobotConfig.driveTrain.left_talons.velocity_kd_high;
      right_kv = RobotConfig.driveTrain.left_talons.velocity_kv_high;
      right_ka = 0;//RobotConfig.driveTrain.left_talons.velocity_ka_high;
    }

    m_leftFollower = new DistanceFollower(m_leftTrajectory);
    m_rightFollower = new DistanceFollower(m_rightTrajectory);
    m_leftFollower.configurePIDVA(left_kp, left_ki, left_kd, left_kv, left_ka);
    m_rightFollower.configurePIDVA(right_kp, right_ki, right_kd, right_kv, right_ka);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    m_leftOutput = m_leftFollower.calculate(Robot.drivetrain.getLeftDistance() + RobotConfig.driveTrain.left_static_kv);
    m_rightOutput = m_rightFollower.calculate(Robot.drivetrain.getLeftDistance() + RobotConfig.driveTrain.right_static_kv);
    m_angularError = Pathfinder.boundHalfDegrees(Pathfinder.r2d(-m_leftFollower.getHeading()) - Robot.gyro.getAngle());
    
    // TODO make sure that the sign is the correct direction, it should be!
    m_turn = RobotConfig.auto.pathfinder.gyro_correct_kp * m_angularError;
    
    // Robot.drive.addDesiredVelocities(m_leftFollower.getSegment().velocity, m_rightFollower.getSegment().velocity);
    Robot.drivetrain.setPowers(m_leftOutput - m_turn, m_rightOutput + m_turn);
    SmartDashboard.putNumber("Velocity", m_leftFollower.getSegment().velocity);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return m_rightFollower.isFinished() && m_leftFollower.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // Robot.drive.setPowerZero();
    Robot.drivetrain.setSpeeds(0,0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}