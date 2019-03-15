package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;

/**
 * an interface for an auto mode chooser
 */
public interface iAutoChooser {
	public Command getSelection();

	public void startSelected();

	public void addChoice(String name, Command toAdd);

	public void setDefaultChoice(String name, Command default_);

	public void cancelSelected();
}
