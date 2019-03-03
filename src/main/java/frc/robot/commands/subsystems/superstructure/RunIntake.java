/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.superstructure;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.subsystems.superstructure.SuperStructure;

/**
 * Add your docs here.
 */
public class RunIntake extends TimedCommand {
  /**
   * Run the intake at some speeds for a time
   */
  double hatch, cargo;
  public RunIntake(double hatchSpeed, double cargoSpeed, double timeout) {
    super(timeout);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(SuperStructure.intake);
    this.hatch = hatchSpeed;
    this.cargo = cargoSpeed;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    SuperStructure.intake.setCargoSpeed(cargo);
    SuperStructure.intake.setHatchSpeed(hatch);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Called once after timeout
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
