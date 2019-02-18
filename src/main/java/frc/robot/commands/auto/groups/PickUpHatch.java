package frc.robot.commands.auto.groups;

import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.RobotConfig;
import frc.robot.commands.auto.actions.DriveDistance;
import frc.robot.commands.subsystems.superstructure.SetHatchMech;
import frc.robot.commands.subsystems.superstructure.SuperstructureGoToState;
import frc.robot.states.ElevatorState;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.Intake.HatchMechState;
import frc.robot.subsystems.superstructure.SuperStructure;

/** 
 * runs a series of commands to pick up a hatch from the loading station
 * FIXME test irl
 */
public class PickUpHatch extends CommandGroup {
	public PickUpHatch() {
		/* The plan right now is to lower the elevator, drive to
		 a distance from the loading station based on the Lidar, 
		 move the elevator up while intaking and once up to a 
		set height start slowly backing up.
		*/
		// rams into the loading station (hopefully)
		// addSequential(new FollowVisionTarget(0.6, 100, 20)); //FIXME percent frame check
		// grabs the hatch by opening the clamp
		addSequential(new SetHatchMech(HatchMechState.kClamped));
		// lifts the hatch out of the brushes
		addSequential(new SuperstructureGoToState(new SuperStructureState(new ElevatorState((LengthKt.getInch(RobotConfig.auto.fieldPositions.hatchLowGoal.getInch() + 5))), SuperStructure.getInstance().getCurrentState().getElbow(), SuperStructure.getInstance().getCurrentState().getWrist()))); //cs hatch is same as loading station
		// moves the robot back slightly
		addSequential(new DriveDistance(-1, 20)); // FIXME check values
		//robot returns to operator control
	}

}
