/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.superstructure;

import org.team5940.pantry.exparimental.command.InstantCommand;

import frc.robot.subsystems.superstructure.Elevator.ElevatorGear;
import frc.robot.subsystems.superstructure.SuperStructure;

public class SetElevatorGear extends InstantCommand {

	private ElevatorGear mReq;

	public SetElevatorGear(ElevatorGear mReq_) {
		// Use addRequirements() here to declare subsystem dependencies
		// eg. addRequirements(chassis);
		this.mReq = mReq_;
	}

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		SuperStructure.elevator.setGear(mReq);
	}

}
