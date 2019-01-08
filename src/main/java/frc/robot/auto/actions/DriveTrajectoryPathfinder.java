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

public class DriveTrajectoryPathfinder extends Command {
  private Trajectory m_leftTrajectory, m_rightTrajectory, m_sourceTrajectory;
  private DistanceFollower m_leftFollower, m_rightFollower;
  private double m_leftOutput, m_rightOutput, m_turn, m_angularError;
  private TankModifier m_modifier;
  /** The start distances of the drivetrain on robot init */
  private double leftStartDistance, rightStartDistance;
  /** Gyro variables for turn P */
  private double desired_heading;

  double left_kp, left_ki, left_kd, left_kv, left_ka, right_kp, right_ki, right_kd, right_kv, right_ka;


  public DriveTrajectoryPathfinder(Trajectory traj) {
    m_sourceTrajectory = traj;
    m_modifier = new TankModifier(m_sourceTrajectory);
    m_modifier.modify(RobotConfig.driveTrain.drivetrain_width);    
    m_leftTrajectory = m_modifier.getLeftTrajectory();
    m_rightTrajectory = m_modifier.getRightTrajectory();
    requires(Robot.drivetrain);
  }

  public DriveTrajectoryPathfinder(String file) {
    requires(Robot.drivetrain);

    File traj = new File("/home/lvuser/deploy/paths/test.pf1.csv");
    Trajectory m_sourceTrajectory = Pathfinder.readFromCSV(traj);
    File leftTraj = new File("/home/lvuser/deploy/paths/test.left.pf1.csv");
    Trajectory m_leftTrajectory = Pathfinder.readFromCSV(leftTraj);
    File rightTraj = new File("/home/lvuser/deploy/paths/test.right.pf1.csv");
    Trajectory m_rightTrajectory = Pathfinder.readFromCSV(rightTraj);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    if(RobotConfig.auto.auto_gear == Gear.LOW){
      left_kp = RobotConfig.driveTrain.left_talons.velocity_kp_low;
      left_ki = 0;//RobotConfig.driveTrain.left_talons.velocity_ki_low;
      left_kd = RobotConfig.driveTrain.left_talons.velocity_kd_low;
      left_kv = RobotConfig.driveTrain.left_talons.velocity_kv_low;
      left_ka = 0;//RobotConfig.driveTrain.left_talons.velocity_ki_low;

      right_kp = RobotConfig.driveTrain.left_talons.velocity_kp_low;
      right_ki = 0;//RobotConfig.driveTrain.left_talons.velocity_ki_low;
      right_kd = RobotConfig.driveTrain.left_talons.velocity_kd_low;
      right_kv = RobotConfig.driveTrain.left_talons.velocity_kv_low;
      right_ka = 0;//RobotConfig.driveTrain.left_talons.velocity_ka_low;
    }

    // Modify the variables if the gear is high, yes this is a bit of a hack but 
    else {
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

    leftStartDistance = Robot.drivetrain.getLeftDistance();
    rightStartDistance = Robot.drivetrain.getRightDistance();

    m_leftFollower = new DistanceFollower(m_leftTrajectory);
    m_rightFollower = new DistanceFollower(m_rightTrajectory);
    m_leftFollower.configurePIDVA(left_kp, left_ki, left_kd, left_kv, left_ka);
    m_rightFollower.configurePIDVA(right_kp, right_ki, right_kd, right_kv, right_ka);

    System.out.println("Pathfinder auto init-ed!");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    m_leftOutput = m_leftFollower.calculate(Robot.drivetrain.getLeftDistance() - leftStartDistance) + RobotConfig.driveTrain.left_static_kv;
    m_rightOutput = m_rightFollower.calculate(Robot.drivetrain.getLeftDistance() - rightStartDistance) + RobotConfig.driveTrain.right_static_kv;
    
    desired_heading = Pathfinder.r2d(m_leftFollower.getHeading());
    m_angularError = Pathfinder.boundHalfDegrees(desired_heading - Robot.gyro.getAngle());
        
    // TODO make sure that the sign is the correct direction, it should be!
    m_turn = -RobotConfig.auto.pathfinder.gyro_correct_kp * m_angularError;
    
    Robot.drivetrain.setVoltages(m_leftOutput + m_turn, m_rightOutput - m_turn);

    SmartDashboard.putString("Left pathfinder data: ", 
      String.format("Velocity (position) heading: %s (%s) %s", 
      m_leftFollower.getSegment().velocity, 
      m_leftFollower.getSegment().position, 
      m_leftFollower.getSegment().heading));
    SmartDashboard.putString("Right pathfinder data: ", 
      String.format("Velocity (position) heading: %s (%s) %s", 
      m_rightFollower.getSegment().velocity, 
      m_rightFollower.getSegment().position, 
      m_rightFollower.getSegment().heading));

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