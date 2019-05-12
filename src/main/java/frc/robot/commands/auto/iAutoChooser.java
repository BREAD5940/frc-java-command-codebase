package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.command.Command;

/**
 * an interface for an auto mode chooser
 */
public interface iAutoChooser {
	Command getSelection();

	void startSelected();

	void addChoice(String name, Command toAdd);

	void setDefaultChoice(String name, Command default_);

	void cancelSelected();
}
