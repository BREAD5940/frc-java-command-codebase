// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.commands.subsystems.superstructure;

// import edu.wpi.first.wpilibj.command.Command;
// import edu.wpi.first.wpilibj.command.InstantCommand;
// import frc.robot.lib.obj.RoundRotation2d;
// import frc.robot.states.SuperStructureState;
// import frc.robot.subsystems.superstructure.SuperStructure;

// /**
//  * Add your docs here.
//  */
// public class JogElbow extends Command {
// 	RoundRotation2d deltaAngle;
// 	boolean isUpwards;
// 	Command moveCommand;

// 	/**
// 	 * Add your docs here.
// 	 */
// 	public JogElbow(RoundRotation2d delta) {
// 		super();
// 		// Use requires() here to declare subsystem dependencies
// 		// eg. requires(chassis);
// 		requires(SuperStructure.getInstance());
// 		requires(SuperStructure.getInstance().getElbow());
// 		requires(SuperStructure.getInstance().getWrist());
// 		requires(SuperStructure.getElevator());

// 		deltaAngle = delta;
// 		// isUpwards = isUp;
// 	}

// 	// Called once when the command executes
// 	@Override
// 	protected void initialize() {
// 		SuperStructureState currentProximalAngle = SuperStructure.getInstance().getCurrentState();
// 		var newAngle = currentProximalAngle.getElbowAngle().plus(deltaAngle);

// 		System.out.println("changing old angle of "  + currentProximalAngle.getElbowAngle().getDegree() + "to " + newAngle);

// 		SuperStructure.getInstance().getElbow().getMaster().setSensorPosition(newAngle);

// 		System.out.println("angle is now " + SuperStructure.getInstance().getElbow().getMaster().getSensorPosition());

// 		moveCommand = new ArmMove(currentProximalAngle.jointAngles);
// 		moveCommand.start();
// 		// updateStateCommand.clearRequirements();

// 	}

// 	@Override
// 	protected boolean isFinished() {
// 		if(moveCommand == null) return true;
// 		return moveCommand.isCompleted();
// 	}

// }
