package frc.robot.commands.subsystems.superstructure;

import org.ghrobotics.lib.mathematics.units.Length;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.states.ElevatorState;
import frc.robot.states.IntakeAngle;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class JankyGoToState extends CommandGroup {

	public JankyGoToState(Length height, IntakeAngle angles) {
		this(new SuperStructureState(new ElevatorState(height), angles));
	}

	public JankyGoToState(ElevatorState height, IntakeAngle angles) {
		this(new SuperStructureState(height, angles));
	}

	public JankyGoToState(SuperStructureState requ_) {

		addSequential(new ConditionalCommand(new ElevatorThanArm(requ_), new ArmThanElevator(requ_)){
		
			@Override
			protected boolean condition() {
				var currentState = SuperStructure.getInstance().getCurrentState();
				var startAboveSafe = requ_.getElevatorHeight().getInch() > 25;
				var endAboveSafe = currentState.getElevatorHeight().getInch() > 25;
				var nowOutsideFrame = currentState.getElbowAngle().getDegree() > -50;
				var willBeOutsideFrame = requ_.getElbowAngle().getDegree() > -50;

				var shouldMoveElevatorFirst = (nowOutsideFrame && !willBeOutsideFrame && !startAboveSafe) || (nowOutsideFrame && willBeOutsideFrame);

				return shouldMoveElevatorFirst;

			}
		});


	}


	public class ElevatorThanArm extends CommandGroup {
		public ElevatorThanArm(SuperStructureState requ) {
			addSequential(new ElevatorMove(requ.getElevator()));			
			addSequential(new ArmMove(requ.getAngle()));
		}
	}

	public class ArmThanElevator extends CommandGroup {
		public ArmThanElevator(SuperStructureState requ) {
			addSequential(new ArmMove(requ.getAngle()));
			addSequential(new ElevatorMove(requ.getElevator()));
		}
	}

}
