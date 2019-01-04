package frc.math;

public class coordinateSystems {
    double[] xyArray = new double[2];
    
    /** 
     * Convert from polar to cartesian
     * @param double[] in the form theta, magnitude
     * @return a double[] in the form x, y
     */
    public static double[] polarToCartesian(double[] polar) {
        polar[0] = Math.toRadians(polar[0]); // Convert the angle to radians because that's waht sin/cox expect
        double[] xyArray = new double[2];
        xyArray[0] = polar[1] * ( Math.cos(polar[0]) );
        xyArray[1] = polar[1] * ( Math.sin(polar[0]) );
        return xyArray;
    }
}