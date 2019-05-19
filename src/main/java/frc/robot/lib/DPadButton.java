package frc.robot.lib;

//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.buttons.Button;

import org.team5940.pantry.exparimental.buttons.Button;

import edu.wpi.first.wpilibj.Joystick;

public class DPadButton extends Button {

	Joystick joystick;
	Direction direction;

	public DPadButton(Joystick joystick, Direction direction) {
		this.joystick = joystick;
		this.direction = direction;
	}

	public static enum Direction {
		UP(0), RIGHT(90), DOWN(180), LEFT(270);

		int direction;

		private Direction(int direction) {
			this.direction = direction;
		}
	}

	@Override
	public boolean get() {
		int dPadValue = joystick.getPOV();
		return (dPadValue == direction.direction) || (dPadValue == (direction.direction + 45) % 360)
				|| (dPadValue == (direction.direction + 315) % 360);
	}

}
