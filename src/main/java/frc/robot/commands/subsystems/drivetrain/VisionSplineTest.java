/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.drivetrain;

import java.util.Arrays;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotConfig.driveTrain;
import frc.robot.commands.auto.Trajectories;
import frc.robot.lib.motion.Util;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;

public class VisionSplineTest extends Command {
  public VisionSplineTest() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(DriveTrain.getInstance());
  }

  boolean mCommandStarted = false;
  TimedTrajectory<Pose2dWithCurvature> trajectory;
  Command mFollowerCommand;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    Pose2d mVisionTargetPose = LimeLight.getInstance().getPose();

    double now = Timer.getFPGATimestamp();
    final double kOffset = 200;
    // Pose2d start = new Pose2d(new Translation2d(LengthKt.getInch(-50 + 200), LengthKt.getInch(-20 + 200)), Rotation2dKt.getDegree(-10));
    Pose2d end = new Pose2d(new Translation2d(LengthKt.getInch(0+kOffset), LengthKt.getInch(0+kOffset)), Rotation2dKt.getDegree(0));

    trajectory = Trajectories.generateTrajectoryLowGear(Arrays.asList(mVisionTargetPose, end), false);

    mFollowerCommand = DriveTrain.getInstance().followTrajectoryWithGear(trajectory, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true);
    
    System.out.println("========== Vision spline generated in " + (Timer.getFPGATimestamp() - now) + " seconds! ==========");

    System.out.println("Pose2d of the target that we measured: " + Util.toString(mVisionTargetPose));

    this.clearRequirements();
    mFollowerCommand.start();
    mCommandStarted = true;
  
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // if(mCommandStarted && mFollowerCommand.isCompleted()) requires(DriveTrain.getInstance());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return mCommandStarted && mFollowerCommand.isCompleted();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    DriveTrain.getInstance().stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
