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
import frc.robot.lib.obj.RoundRotation2d;
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
	private static final Length kZeroHeight = LengthKt.getInch(21.5);
	private Length mZeroHeight;

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		mCurrentState = ZeroingState.IDLE;

		SmartDashboard.putBoolean("Elevator zeroed", false);

		SmartDashboard.putBoolean("Proximal zeroed", false);

		SmartDashboard.putBoolean("Wrist zeroed", false);

		SmartDashboard.putData(this);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {

		var limitTriggered = SuperStructure.getInstance().getInnerStageMinLimit();

		SmartDashboard.putString("Zeroing state", mCurrentState.name());
		SmartDashboard.putBoolean("Elevator limit switch", limitTriggered);

		if (!DriverStation.getInstance().isDisabled())
			return;
		// switch to observe desired behavior

		if (mCurrentState == ZeroingState.IDLE) {
			// System.out.println("in idle state");
			// var limitTriggered = limitStatus;
			if (!limitTriggered) {
				mCurrentState = ZeroingState.WAITING_FOR_TRIGGER;
				// System.out.println("limit switch is off, waiting for retrigger");
				// break;
			}
		} else if (mCurrentState == ZeroingState.WAITING_FOR_TRIGGER) {
			// System.out.println("waiting for trigger");
			// limitTriggered = limitStatus;
			if (limitTriggered) {
				// System.out.println("observing elevator zeroed");
				observeELevatorZeroed();
				mCurrentState = ZeroingState.ZEROED;
				// break;
			}
		}
		// else return;
	}
	// }

	protected void observeELevatorZeroed() {

		SuperStructure.getInstance().getElbow().getMaster().set(ControlMode.PercentOutput, 0);
		SuperStructure.getInstance().getWrist().getMaster().set(ControlMode.PercentOutput, 0);

		SmartDashboard.putBoolean("Elevator zeroed", true);

		SmartDashboard.putBoolean("Proximal zeroed", true);

		SmartDashboard.putBoolean("Wrist zeroed", true);

		var proximal = SuperStructure.getInstance().getElbow();
		// var startingAngleTicks = (int) proximal.getMaster().getTicks(RoundRotation2d.getDegree(-90)) + (-640) + (proximal.getMaster().getSensorCollection().getPulseWidthPosition() % 2048 * Math.signum(proximal.getMaster().getSensorCollection().getPulseWidthPosition() % 2048));
		// var tickkkkks = (SuperStructure.getInstance().getElbow().getMaster().getSensorCollection().getPulseWidthPosition() % 2048) * ((SuperStructure.getInstance().getElbow().getMaster().getSensorCollection().getPulseWidthPosition() > 0) ? 1 : -1);
		 var targetProximal_ = 1400;
		 var targetProximal_COMP = 1900;
		// var delta = (tickkkkks - (int) targetProximal_COMP) * -1;
		var startingAngleTicks = proximal.getMaster().getTicks(RoundRotation2d.getDegree(-78));

		proximal.getMaster().setSelectedSensorPosition((int) (0 + startingAngleTicks));
		// proximal.getMaster().setSelectedSensorPosition((int) (startingAngleTicks));

		var wrist = SuperStructure.getInstance().getWrist();
		var wristStart = (int) wrist.getMaster().getTicks(RoundRotation2d.getDegree(-43 + 4 - 9));
		var targetWrist = (int) 1000;
		var targetWristComp = 1500 + 150;
		var correctionDelta = (SuperStructure.getInstance().getElbow().getMaster().getSensorCollection().getPulseWidthPosition() % 2048) * ((SuperStructure.getInstance().getElbow().getMaster().getSensorCollection().getPulseWidthPosition() > 0) ? 1 : -1);

//		var deltaW = (correctionDelta - (int) targetWrist) * 1;
		var deltaW = (correctionDelta - (int) targetWristComp) * 1;


		wrist.getMaster().setSelectedSensorPosition((int) (deltaW + wristStart));

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
		SuperStructure.elevator.elevatorZeroed = true;
		SmartDashboard.putString("Zeroing state", mCurrentState.name());
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		SmartDashboard.putString("Zeroing state", mCurrentState.name());
	}
}
