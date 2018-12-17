package frc.robot;

public interface xboxmap {

	public interface Axis {
		public static final int LEFT_JOYSTICK_X = 0;
		
		public static final int LEFT_JOYSTICK_Y = 1;

		public static final int LEFT_TRIGGER = 2;

		public static final int RIGHT_TRIGGER = 3;

		public static final int RIGHT_JOYSTICK_X = 4;
		
		public static final int RIGHT_JOYSTICK_Y = 5;
	}

	public interface Buttons {
		public static final int A_BUTTON = 1;

		public static final int B_BUTTON = 2;

		public static final int X_BUTTON = 3;

		public static final int Y_BUTTON = 4;

		public static final int LB_BUTTON = 5;

		public static final int RB_BUTTON = 6;

		/**
		 * This is the small buttons right by the xBox logo. I am not sure of
		 * the actual name.
		 */
		public static final int LEFT_START_BUTTON = 7;

		/**
		 * See above comment.
		 */
		public static final int RIGHT_START_BUTTON = 8;

		/**
		 * Pressing the left Joystick in.
		 */
		public static final int LEFT_JOYSTICK_BUTTON = 9;

		/**
		 * Pressing the right Joystick in.
		 */
		public static final int RIGHT_JOYSTICK_BUTTON = 10;
	}

}
