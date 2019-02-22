package frc.robot.lib;

import java.util.List;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class ChordedInput extends Button {

	List<JoystickButton> buttons;

	public ChordedInput(List<JoystickButton> buttons) {
		this.buttons = buttons;
	}

	@Override
	public boolean get() {
		boolean ifAllPressed = true;
		for (JoystickButton b : buttons) {
			ifAllPressed = ifAllPressed && b.get();
		}
		return ifAllPressed;
	}

}
