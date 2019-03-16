package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.auto.routines.CargoShip1;
// import frc.robot.commands.auto.routines.CargoShipLeft;
import frc.robot.commands.auto.routines.CloseSideRocket;
import frc.robot.commands.auto.routines.FarSideRocket;

/**
 * Literally just a sendable chooser
 */
public class TerribleAutoChooser implements iAutoChooser {

	private SendableChooser<Command> mAutoChooser;

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
		addChoice("HabR to cargoR1", new CloseSideRocket('R'));
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