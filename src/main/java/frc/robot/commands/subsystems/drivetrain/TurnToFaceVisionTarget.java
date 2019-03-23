/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.drivetrain;

import java.util.TreeMap;

import com.team254.lib.physics.DifferentialDrive.ChassisState;

import org.ghrobotics.lib.mathematics.units.Rotation2d;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.lib.InterpolatableLut;
import frc.robot.lib.InterpolatableLutEntry;
import frc.robot.lib.motion.Util;
import frc.robot.lib.InterpolatableLut;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.LimeLight;

public class TurnToFaceVisionTarget extends Command {

  private InterpolatableLut skewCorrection;

  private double targetAngle;

  public TurnToFaceVisionTarget() {
    requires(DriveTrain.getInstance());

    var map = new TreeMap<Double, InterpolatableLutEntry>();
    map.put(Double.valueOf(0), new InterpolatableLutEntry(0));

    skewCorrection = new InterpolatableLut(map);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    var targetX = LimeLight.getInstance().getDx().getDegree();
    var robotYaw = DriveTrain.getInstance().getGyro();
    var interpolationOffset = skewCorrection.interpolate(LimeLight.getInstance().getTargetSkew());

    targetAngle =  targetX + robotYaw + interpolationOffset;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    final double kp = 0.3;

    var error = DriveTrain.getInstance().getGyro() - targetAngle;
    var turnPower = kp * error;

    turnPower = Util.limit(turnPower, 4);

    ChassisState state = new ChassisState(0, turnPower);

    DriveTrain.getInstance().setOutputFromKinematics(state);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    var error = DriveTrain.getInstance().getGyro() - targetAngle;

    return Math.abs(error) < 2;
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
