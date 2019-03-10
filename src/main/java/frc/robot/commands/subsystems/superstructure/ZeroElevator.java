/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.superstructure;

import org.ghrobotics.lib.mathematics.units.Length;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Robot.RobotState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class ZeroElevator extends Command {
	Length topOfInnerStage;
	boolean hasFoundZero = false;

	public ZeroElevator(Length zeroHeight) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		// requires(SuperStructure.elevator);
		this.topOfInnerStage = zeroHeight;
		setRunWhenDisabled(true);
	}

	// Called just before this Command runs the first time
	@Override
	protected void execute() {
		var topLimitTriggered = SuperStructure.getInstance().getCarriageMaxLimit();
		var bottomLimitTriggered = SuperStructure.getInstance().getInnerStageMinLimit();
		if (topLimitTriggered && bottomLimitTriggered) {
			setZero();
			hasFoundZero = true;
		}
	}

	private void setZero() {
		SuperStructure.elevator.getMaster().setSensorPosition(topOfInnerStage);
	}

	@Override
	protected boolean isFinished() {
		var currentState = Robot.getState();
		var isDisabled = (currentState == RobotState.DISABLED);
		return hasFoundZero || !isDisabled;
	}

}
