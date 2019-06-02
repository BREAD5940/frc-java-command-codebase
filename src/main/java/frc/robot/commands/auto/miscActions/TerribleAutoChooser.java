package frc.robot.commands.auto.miscActions;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.auto.routines.*;

/**
 * Literally just a sendable chooser
 */
public class TerribleAutoChooser implements iAutoChooser {

	private SendableChooser<Command> mAutoChooser;

	public SendableChooser<Command> getChooser() {
		return mAutoChooser;
	}

	public TerribleAutoChooser() {
		mAutoChooser = new SendableChooser<Command>();
		mAutoChooser.setName("Autonomous chooser");
	}

	public void addOptions() {
				addChoice("HabL to rocketLF", new FarSideRocket('L'));
				addChoice("HabR to rocketRF", new FarSideRocket('R'));
				addChoice("HabL to rocketLC", new CloseSideRocket('L'));
				addChoice("HabR to rocketRC", new CloseSideRocket('R'));
				addChoice("HabL to cargoL1", new CargoShip1('L'));
				addChoice("HabR to cargoR1", new CargoShip1('R'));
				addChoice("HabL to TWO HATCH AUTO", new CloseThenFarRocket('L'));
				addChoice("HabR to TWO HATCH AUTO", new CloseThenFarRocket('R'));
				addChoice("Baseline", new Baseline());
		setDefaultChoice("Do nothing", new CommandGroup("nothing"));
	}

	@Override
	public Command getSelection() {
		return mAutoChooser.getSelected();
	}

	@Override
	public void startSelected() {
		getSelection().start();
	}

	@Override
	public void addChoice(String name, Command toAdd) {
		mAutoChooser.addOption(name, toAdd);
	}

	@Override
	public void setDefaultChoice(String name, Command default_) {
		mAutoChooser.setDefaultOption(name, default_);
	}

	@Override
	public void cancelSelected() {
		getSelection().cancel();
	}

}
