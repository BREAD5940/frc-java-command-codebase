package frc.robot;

import edu.wpi.first.wpilibj.buttons.Button;

import edu.wpi.first.wpilibj.buttons.JoystickButton;

public interface dhdMap {

	public static final int RUN = 30;

	public interface Start {
		public static final Button L =new JoystickButton(OI.dhd, 0);

		public static final Button M = new JoystickButton(OI.dhd,1);

		public static final Button R = new JoystickButton(OI.dhd,2);

		public static final Button ONE = new JoystickButton(OI.dhd,3);

		public static final Button TWO = new JoystickButton(OI.dhd,4);

		public static final Button THREE = new JoystickButton(OI.dhd,5);

		public static final Button HAB = new JoystickButton(OI.dhd,6);

		public static final Button LOAD = new JoystickButton(OI.dhd,7);

		public static final Button CARGO = new JoystickButton(OI.dhd,8);

		public static final Button ROCKET = new JoystickButton(OI.dhd,9);

		public static final Button DEPOT = new JoystickButton(OI.dhd,10);
	}

	public interface End {
		public static final Button L = new JoystickButton(OI.dhd,11);

		public static final Button M = new JoystickButton(OI.dhd,12);

		public static final Button R = new JoystickButton(OI.dhd,13);

		public static final Button ONE = new JoystickButton(OI.dhd,14);

		public static final Button TWO = new JoystickButton(OI.dhd,15);

		public static final Button THREE = new JoystickButton(OI.dhd,16);

		public static final Button HAB = new JoystickButton(OI.dhd,17);

		public static final Button LOAD = new JoystickButton(OI.dhd,18);

		public static final Button CARGO = new JoystickButton(OI.dhd,19);

		public static final Button ROCKET = new JoystickButton(OI.dhd,20);

		public static final Button DEPOT = new JoystickButton(OI.dhd,21);
	}

	public interface Motion{
		public static final Button LOW = new JoystickButton(OI.dhd,22);

		public static final Button MID = new JoystickButton(OI.dhd,23);
		
		public static final Button HIGH = new JoystickButton(OI.dhd,24);
		
		public static final Button OVER = new JoystickButton(OI.dhd,25);
		
		public static final Button PLACE_HATCH = new JoystickButton(OI.dhd,26);
		
		public static final Button PLACE_CARGO = new JoystickButton(OI.dhd,27);
		
		public static final Button GRAB_HATCH = new JoystickButton(OI.dhd,28);
		
		public static final Button GRAB_CARGO = new JoystickButton(OI.dhd,29);
	}

}
