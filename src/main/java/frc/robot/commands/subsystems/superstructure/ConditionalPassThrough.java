package frc.robot.commands.subsystems.superstructure;

import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.robot.subsystems.superstructure.SuperStructure;

public class ConditionalPassThrough extends ConditionalCommand {

	public ConditionalPassThrough() {

		super(
				new PassThrough.FrontToBack(SuperStructure.getInstance()),
				new PassThrough.BackToFront(SuperStructure.getInstance()));

	}

	@Override
	protected boolean condition() {
		var state = SuperStructure.getInstance().getCurrentState().getElbowAngle().getDegree();
		return state > -90;
	}
}
