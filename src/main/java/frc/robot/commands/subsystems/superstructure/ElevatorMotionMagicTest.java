package frc.robot.commands.subsystems.superstructure;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.states.ElevatorState;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.RotatingJoint.RotatingArmState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class ElevatorMotionMagicTest extends Command {
  public ElevatorMotionMagicTest() {
    requires(SuperStructure.getInstance());
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Length mTarget = LengthKt.getFeet(2);
    double volts = Elevator.getVoltage(new SuperStructureState(new ElevatorState(mTarget), new RotatingArmState(), new RotatingArmState()));
    SuperStructure.getElevator().requestClosedLoop(ControlMode.MotionMagic, mTarget, DemandType.ArbitraryFeedForward, volts);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
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
