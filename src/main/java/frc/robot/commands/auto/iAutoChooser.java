package frc.robot.commands.auto;

import org.team5940.pantry.experimental.command.Command;

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
