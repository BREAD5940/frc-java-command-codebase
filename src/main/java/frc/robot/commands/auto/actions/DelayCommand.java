// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.commands.auto.actions;

// import org.ghrobotics.lib.mathematics.units.Time;

// import org.team5940.pantry.exparimental.command.SendableCommandBase;

// public class DelayCommand extends SendableCommandBase {
// 	final double time;

// 	public DelayCommand(Time time) {
// 		this.time = time.getSecond();
// 	}

// 	// Called just before this Command runs the first time
// 	@Override
// 	public void initialize() {
// 		setTimeout(time);
// 	}

// 	// Called repeatedly when this Command is scheduled to run
// 	@Override
// 	public void execute() {}

// 	// Make this return true when this Command no longer needs to run execute()
// 	@Override
// 	public boolean isFinished() {
// 		return isTimedOut();
// 	}

// 	// Called once after isFinished returns true
// 	@Override
// 	public void end(boolean interrupted) {
// 		System.out.println("Wait command done!");
// 	}

// 	// Called when another command which requires one or more of the same
// 	// subsystems is scheduled to run
// 	@Override
// 	protected void interrupted() {}
// }
