package frc.robot.commands.auto.actions;

import java.io.File;

import frc.robot.Robot;
import frc.robot.RobotConfig;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class DriveTrajectoryPathfinder extends Command {
  private Trajectory mLeftTrajectory;

  private Trajectory mRightTrajectory;

  private Trajectory mSourceTrajectory;

  private DistanceFollower mLeftFollower;
  
  private DistanceFollower mRightFollower;

  private double mLeftOutput;
  private double mRightOutput;
  private double mTurn;
  private double mAngularError;

  private TankModifier mModifier;

  /** The start distances of the drivetrain on robot init */
  private double mLeftStartDistance;
  private double mRightStartDistance;

  /** Gyro variables for turn P */
  private double mDesiredHeading;

  double mLeftKp;

  double mLeftKi;

  double mLeftKd;

  double mLeftKv;

  double mLeftKa;

  double mRightKp;

  double mRightKi;

  double mRightKd;

  double mRightKv;

  double mRightKa;




  public DriveTrajectoryPathfinder(Trajectory mTraj) {
    mSourceTrajectory = mTraj;

    mModifier = new TankModifier(mSourceTrajectory);

    mModifier.modify(RobotConfig.driveTrain.drivetrain_width);    
    mLeftTrajectory = mModifier.getLeftTrajectory();
    mRightTrajectory = mModifier.getRightTrajectory();
    
    requires(Robot.drivetrain);
  }

  public DriveTrajectoryPathfinder(String mFile) {
    requires(Robot.drivetrain);
    
    // So I flipped this because Pathweaver seems to be exporting the left and right
    // flipped for some reason. so fix dis. k thx
    File traj = new File("/home/lvuser/deploy/paths/test.pf1.csv");
    mSourceTrajectory = Pathfinder.readFromCSV(traj);
    // File leftTraj = new File("/home/lvuser/deploy/paths/test.left.pf1.csv");
    // mLeftTrajectory = Pathfinder.readFromCSV(leftTraj);
    // File rightTraj = new File("/home/lvuser/deploy/paths/test.right.pf1.csv");
    // mRightTrajectory = Pathfinder.readFromCSV(rightTraj);
    File leftTraj = new File("/home/lvuser/deploy/paths/test.left.pf1.csv");
    File rightTraj = new File("/home/lvuser/deploy/paths/test.right.pf1.csv");
    mRightTrajectory = Pathfinder.readFromCSV(leftTraj);
    mLeftTrajectory = Pathfinder.readFromCSV(rightTraj);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    // if(RobotConfig.auto.auto_gear == Gear.LOW){
    //   mLeftKp = RobotConfig.driveTrain.left_talons.velocity_kp_low;
    //   mLeftKi = 0;//RobotConfig.driveTrain.left_talons.velocity_ki_low;
    //   mLeftKd = RobotConfig.driveTrain.left_talons.velocity_kd_low;
    //   mLeftKv = RobotConfig.driveTrain.left_talons.velocity_kv_low;
    //   mLeftKa = 0;//RobotConfig.driveTrain.left_talons.velocity_ki_low;

    //   mRightKp = RobotConfig.driveTrain.left_talons.velocity_kp_low;
    //   mRightKi = 0;//RobotConfig.driveTrain.left_talons.velocity_ki_low;
    //   mRightKd = RobotConfig.driveTrain.left_talons.velocity_kd_low;
    //   mRightKv = RobotConfig.driveTrain.left_talons.velocity_kv_low;
    //   mRightKa = 0;//RobotConfig.driveTrain.left_talons.velocity_ka_low;
    // }

    // // Modify the variables if the gear is high, yes this is a bit of a hack but 
    // else {
    mLeftKp = 3; //RobotConfig.driveTrain.left_talons.velocity_kp_high;
    mLeftKi = 0; //RobotConfig.driveTrain.left_talons.velocity_ki_high;
    mLeftKd = 1; //RobotConfig.driveTrain.left_talons.velocity_kd_high;
    mLeftKv = 1.0 / 13; //RobotConfig.driveTrain.left_talons.velocity_kv_high;
    mLeftKa = 0; //RobotConfig.driveTrain.left_talons.velocity_ki_high;

    mRightKp = mLeftKp; //RobotConfig.driveTrain.left_talons.velocity_kp_high;
    mRightKi = mLeftKi; //RobotConfig.driveTrain.left_talons.velocity_ki_high;
    mRightKd = mLeftKd; //RobotConfig.driveTrain.left_talons.velocity_kd_high;
    mRightKv = mLeftKv; //RobotConfig.driveTrain.left_talons.velocity_kv_high;
    mRightKa = 0; //RobotConfig.driveTrain.left_talons.velocity_ka_high;
    // }

    mLeftStartDistance = Robot.drivetrain.getLeftDistance();
    mRightStartDistance = Robot.drivetrain.getRightDistance();

    mLeftFollower = new DistanceFollower(mLeftTrajectory);
    mRightFollower = new DistanceFollower(mRightTrajectory);
    mLeftFollower.configurePIDVA(mLeftKp, mLeftKi, mLeftKd, mLeftKv, mLeftKa);
    mRightFollower.configurePIDVA(mRightKp, mRightKi, mRightKd, mRightKv, mRightKa);

    Robot.gyro.reset();

    System.out.println("Pathfinder auto init-ed!");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    try {
      mLeftOutput = mLeftFollower.calculate(Robot.drivetrain.getLeftDistance() - mLeftStartDistance) + RobotConfig.driveTrain.left_static_kv;
      mRightOutput = mRightFollower.calculate(Robot.drivetrain.getRightDistance() - mRightStartDistance) + RobotConfig.driveTrain.right_static_kv;
    } catch (ArrayIndexOutOfBoundsException e) {
      mLeftOutput = 0;
      mRightOutput = 0;
    }
    
    mDesiredHeading = Pathfinder.r2d(mLeftFollower.getHeading());
    mAngularError = Pathfinder.boundHalfDegrees(mDesiredHeading - Robot.gyro.getAngle());
        
    // TODO make sure that the sign is the correct direction, it should be!
    mTurn = mAngularError * RobotConfig.auto.pathfinder.gyro_correct_kp;
    
    Robot.drivetrain.setVoltages(mLeftOutput + mTurn, mRightOutput - mTurn);

    SmartDashboard.putString("Left target pathfinder data: ", 
      String.format("Velocity (position) heading (current): %s (%s) %s (%s)", 
      mLeftFollower.getSegment().velocity, 
      mLeftFollower.getSegment().position, 
      mLeftFollower.getSegment().heading,
      Robot.drivetrain.getLeftDistance()));
    SmartDashboard.putString("Right target pathfinder data: ", 
      String.format("Velocity (position) heading (current): %s (%s) %s (%s)", 
      mRightFollower.getSegment().velocity, 
      mRightFollower.getSegment().position, 
      mRightFollower.getSegment().heading,
      Robot.drivetrain.getRightDistance()));

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return mRightFollower.isFinished() && mLeftFollower.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // Robot.drive.setPowerZero();
    Robot.drivetrain.setSpeeds(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
