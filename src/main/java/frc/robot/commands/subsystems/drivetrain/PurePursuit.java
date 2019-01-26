package frc.robot.commands.subsystems.drivetrain;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.Arrays;
import java.util.List;

import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.lib.Logger;
import frc.robot.lib.motion.Odometer;
import frc.robot.lib.motion.followers.PathFollower;
import frc.robot.lib.motion.purepursuit.Path;
import frc.robot.lib.motion.purepursuit.PathGenerator;
import frc.robot.lib.motion.purepursuit.Point;
import frc.robot.lib.motion.purepursuit.WayPoint2;
import frc.robot.lib.obj.DriveSignal;

public class PurePursuit extends Command {
  PathFollower pathFollower;
  Path path = null;
  double dt = 0.02;

  public PurePursuit() {
    Robot.drivetrain.gyro.reset();
    Robot.drivetrain.zeroEncoders();

    requires(Robot.drivetrain);

    Robot.drivetrain.zeroGyro();

  }

  @Override
  protected void initialize() {
    initPath();
    setTimeout(10);
    
  }

  @Override
  public void execute() {
    List<DriveSignal> driveMotorStates = pathFollower.update(Odometer.getInstance().getPoint(), Robot.drivetrain.getGyro(), dt);

    // updateSmartDashboard();

    SmartDashboard.putNumber("Left Vel", round(VelocityKt.getFeetPerSecond(driveMotorStates.get(0).getLeft())));
    SmartDashboard.putNumber("Right Vel", round(VelocityKt.getFeetPerSecond(driveMotorStates.get(0).getRight())));

    Logger.log("Left Vel" + round(VelocityKt.getFeetPerSecond(driveMotorStates.get(0).getLeft())));
    Logger.log("Right Vel" + round(VelocityKt.getFeetPerSecond(driveMotorStates.get(0).getRight())));

    Logger.log("_____________________________________\n");
    // Robot.drivetrain.setPowers(driveMotorStates.leftVel / 0.8 , driveMotorStates.rightVel / 0.8 );

    DriveSignal velocity = driveMotorStates.get(0);

    Robot.drivetrain.setClosedLoop(velocity);
  }

  @Override
  protected boolean isFinished() {
    return pathFollower.done || isTimedOut() ;
  }

  @Override
  protected void end() {

  }

  @Override
  protected void interrupted() {

  }

  
  private void initPath() {
    // double angle = 0;
    Point pos = new Point();
    double maxVel = 1.5; // 6 feet/s
    double maxAcc = 2; // m/sec every sec
    double spacing = 0.05, maxAngVel = 1; // 3 in simu
    double lookAheadDistance = 0.1524 * 8;
    /* 6 inches */
    double trackWidth = 0.8;// 0.5842; // 23 inches
    double targetTolerance = 0.1;// m
    // double maxVel = 250, maxAcc = 70, spacing = 6, maxAngVel = 6;
    // double lookAheadDistance = 12;

    // double kLeftV = 1, kLeftA = 0, kLeftP = 0, kRightV = 1, kRightA = 0, kRightP
    // = 0, kSmooth = 0.8, kTolerance = 0.001;

    // generate path
    PathGenerator pathGenerator = new PathGenerator();
    // path = pathGenerator
    // .calculate(new Path(spacing, maxVel, maxAcc, maxAngVel, Arrays.asList(new
    // WayPoint2(0, 0), new WayPoint2(0, 2))));
    path = pathGenerator.calculate(new Path(spacing, maxVel, maxAcc, maxAngVel,
    Arrays.asList(new WayPoint2(5.083, 1.354), new WayPoint2(5.649, 3.6576), new WayPoint2(6.766, 4.99),
    /*new WayPoint2(6.37, 6.518),*/ new WayPoint2(5.5, 6.1))));
    // path = pathGenerator.calculate(new Path(spacing, maxVel, maxAcc, maxAngVel,
    //     Arrays.asList(new WayPoint2(0, 0), new WayPoint2(0, 4), new WayPoint2(-2, 4), new WayPoint2(-2, 2),
    //         new WayPoint2(0, 2), new WayPoint2(0, 4), new WayPoint2(-2, 4), new WayPoint2(-2, 2), new WayPoint2(2, 2))));
    // path = pathGenerator.calculate(new Path(spacing, maxVel, maxAcc, maxAngVel,
    // Arrays.asList(new WayPoint2(0, 0), new WayPoint2(0, 4), new WayPoint2(-2, 4),
    // new WayPoint2(-2, 0))));
    // path = pathGenerator.calculate(new Path(spacing, maxVel, maxAcc, maxAngVel,
    // Arrays.asList(new WayPoint2(0, 0), new WayPoint2(0, -4), new WayPoint2(-2, -4),
    // new WayPoint2(-2, 0))));

    Logger.log("Path: [" + path.waypoints.size() + "]");
    for (int i = 0; i < path.waypoints.size(); i++) {
      Logger.log("path", path.waypoints.get(i).toString() );
    }
    Logger.log("__________________________\n\n");
    // end generate path

    pathFollower = new PathFollower(path, lookAheadDistance, trackWidth, targetTolerance);
    pos = path.waypoints.get(0).p.copy();
    Robot.odometry_.setX(pos.x);
    Robot.odometry_.setY(pos.y);
    Robot.drivetrain.zeroGyro();
  }

  public static double round(double value) {
    int places = 2;
    // return value;
    if (places < 0)
      throw new IllegalArgumentException();

    BigDecimal bd = new BigDecimal(value);
    bd = bd.setScale(places, RoundingMode.HALF_UP);
    return bd.doubleValue();
  }

}