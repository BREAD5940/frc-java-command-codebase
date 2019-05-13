package frc.robot.lib;

import edu.wpi.first.wpilibj.Joystick;
import org.team5940.pantry.exparimental.buttons.Button;

public class AnalogButton extends Button {

	final Joystick joystick;
	final int port;
	final double threshold;

	/**
	 * Button based on an analog trigger
	 * @param joystick the joystick to use
	 * @param analogPort the port of the joystick to use
	 * @param threshold the threshold after which to t r i g g e r
	 */
	public AnalogButton(final Joystick joystick, final int analogPort, final double threshold) {
		this.joystick = joystick;
		this.port = analogPort;
		this.threshold = (threshold);
	}

	@Override
	public boolean get() {
		double value = (joystick.getRawAxis(port));
		boolean isBig = (threshold > 0) ? value > threshold : value < threshold;
		return isBig;
	}

}
