/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.superstructure;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

		SmartDashboard.putBoolean("Elevator zeroed", false);

		SmartDashboard.putData(this);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {

		var limitTriggered = SuperStructure.getInnerStageMinLimit();

		SmartDashboard.putString("Zeroing state", mCurrentState.name());
		SmartDashboard.putBoolean("Elevator limit switch", limitTriggered);

		if (!DriverStation.getInstance().isDisabled())
			return;
		// switch to observe desired behavior
		
			if(mCurrentState == ZeroingState.IDLE) {
				System.out.println("in idle state");
				// var limitTriggered = limitStatus;
				if (!limitTriggered) {
					mCurrentState = ZeroingState.WAITING_FOR_TRIGGER;
					System.out.println("limit switch is off, waiting for retrigger");
					// break;
				}}
			else if (mCurrentState == ZeroingState.WAITING_FOR_TRIGGER) {
			System.out.println("waiting for trigger");
				// limitTriggered = limitStatus;
				if (limitTriggered) {
					System.out.println("observing elevator zeroed");
					observeELevatorZeroed();
					mCurrentState = ZeroingState.ZEROED;
					// break;
				}}
			// else return;
		}
	// }

	protected void observeELevatorZeroed() {
		SmartDashboard.putBoolean("Elevator zeroed", true);
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
	protected void end() {}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {}
}
