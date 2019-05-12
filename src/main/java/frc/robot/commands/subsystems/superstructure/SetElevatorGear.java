/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.superstructure;

import org.team5940.pantry.exparimental.command.SendableCommandBase;
import frc.robot.subsystems.superstructure.Elevator.ElevatorGear;
import frc.robot.subsystems.superstructure.SuperStructure;

public class SetElevatorGear extends SendableCommandBase {

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

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		return true;
	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {}


}
