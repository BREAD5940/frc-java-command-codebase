// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.commands.subsystems.superstructure;

// import org.ghrobotics.lib.mathematics.units.Length;

// import edu.wpi.first.wpilibj.command.InstantCommand;
// import frc.robot.subsystems.superstructure.SuperStructure;

// /**
//  * Add your docs here.
//  */
// public class JogElevator extends InstantCommand {
// 	Length deltaLength;
// 	boolean isUpwards;

// 	/**
// 	 * Add your docs here.
// 	 */
// 	public JogElevator(Length delta, boolean isUp) {
// 		super();
// 		// Use requires() here to declare subsystem dependencies
// 		// eg. requires(chassis);
// 		requires(SuperStructure.getInstance());
// 		requires(SuperStructure.getElevator());
// 		deltaLength = delta.getAbsoluteValue();
// 		isUpwards = isUp;
// 	}

// 	// Called once when the command executes
// 	@Override
// 	protected void initialize() {
// 		var cachedState = SuperStructure.getInstance().getCurrentState();
// 		SuperStructure.getElevator().jogHeightTrim(deltaLength, isUpwards);
// 		SuperStructure.getInstance().move(cachedState);
// 	}

// }
