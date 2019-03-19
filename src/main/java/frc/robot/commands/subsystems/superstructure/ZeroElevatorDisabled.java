/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.superstructure;

import com.ctre.phoenix.motorcontrol.ControlMode;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.superstructure.SuperStructure;

public class ZeroElevatorDisabled extends Command {
  public ZeroElevatorDisabled(Length height) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(SuperStructure.getInstance());
    requires(SuperStructure.getElevator());
    requires(SuperStructure.getInstance().getWrist());
    requires(SuperStructure.getInstance().getElbow());
    setInterruptible(true);
    setRunWhenDisabled(true);
    this.mZeroHeight = height;
  }

  public ZeroElevatorDisabled() {
    this(kZeroHeight);
  }

  private enum ZeroingState {
    IDLE, WAITING_FOR_TRIGGER, ZEROED;
  }

  private ZeroingState mCurrentState;
  private static final Length kZeroHeight = LengthKt.getInch(26);  
  private Length mZeroHeight;

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    mCurrentState = ZeroingState.IDLE;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // switch to observe desired behavior
    switch(mCurrentState){
      case IDLE:
        var limitTriggered = SuperStructure.getInstance().getInnerStageMinLimit();
        if(!limitTriggered) {
          mCurrentState = ZeroingState.WAITING_FOR_TRIGGER;
          break;
        }
      case WAITING_FOR_TRIGGER:
        limitTriggered = SuperStructure.getInstance().getInnerStageMinLimit();
        if(limitTriggered) {
          observeELevatorZeroed();
          mCurrentState = ZeroingState.ZEROED;
          break;
        }
      case ZEROED:
        break;
      default:
        break;
  }}

  protected void observeELevatorZeroed() {
    SuperStructure.getElevator().getMaster().set(ControlMode.PercentOutput, 0);
    SuperStructure.getElevator().getMaster().setSensorPosition(mZeroHeight);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (mCurrentState == ZeroingState.ZEROED) || !DriverStation.getInstance().isDisabled();
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
