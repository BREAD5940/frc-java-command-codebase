package frc.robot.lib;

public class Logger {

	public static void log(Object stuff) {
		System.out.println(stuff);
	}

	public static void log(double re, double ree) {
		log(re + ", " + ree);
	}
}
