package frc.robot.lib.motion.purepursuit;

import java.util.Arrays;

public class Test {
	public static void main(String[] args) {
		PathGenerator p = new PathGenerator();

		Path path = p.generate(0.4, 0.01, new Path(10, 100, 10, 3,
		        Arrays.asList(
		        		new PPWaypoint(0, 350), 
		        		new PPWaypoint(100, 350),
		                new PPWaypoint(150, 300), 
		                new PPWaypoint(150, 200),
		                new PPWaypoint(200, 150), 
		                new PPWaypoint(300, 150))));
		
		System.out.println(path);
	}
}
