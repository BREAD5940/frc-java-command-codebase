package frc.robot.commands.subsystems.superstructure;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.superstructure.SuperStructure;

public class SuperStructureTelop extends Command {
  private SuperStructure superStructure; 
  public SuperStructureTelop(SuperStructure struc) {
    // Use requires() here to declare subsystem dependencies
    requires(struc);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Length delta = LengthKt.getInch(Robot.m_oi.getElevatorAxis() * 0.08);
    superStructure.moveSuperstructureElevator(superStructure.getElevator().getMaster().getSensorPosition().plus(delta));
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
