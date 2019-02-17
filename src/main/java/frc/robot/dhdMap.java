package frc.robot;

public interface dhdMap {

	public static final int RUN = 30;

	public interface Start {
		public static final int L = 0;

		public static final int M = 1;

		public static final int R = 2;

		public static final int ONE = 3;

		public static final int TWO = 4;

		public static final int THREE = 5;

		public static final int HAB = 6;

		public static final int LOAD = 7;

		public static final int CARGO = 8;

		public static final int ROCKET = 9;

		public static final int DEPOT = 10;
	}

	public interface End {
		public static final int L = 11;

		public static final int M = 12;

		public static final int R = 13;

		public static final int ONE = 14;

		public static final int TWO = 15;

		public static final int THREE = 16;

		public static final int HAB = 17;

		public static final int LOAD = 18;

		public static final int CARGO = 19;

		public static final int ROCKET = 20;

		public static final int DEPOT = 21;
	}

	public interface Motion{
		public static final int LOW = 22;

		public static final int MID = 23;
		
		public static final int HIGH = 24;
		
		public static final int OVER = 25;
		
		public static final int PLACE_HATCH = 26;
		
		public static final int PLACE_CARGO = 27;
		
		public static final int GRAB_HATCH = 28;
		
		public static final int GRAB_CARGO = 29;
	}

}
