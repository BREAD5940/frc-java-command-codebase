
package frc.robot.lib;
import edu.wpi.first.wpilibj.command.Subsystem;

public class encoderlib extends Subsystem {
    public encoderlib() {
        super("encoderlib");
    }

    /**
     * Converts raw sensor position to rotations. Units are full rotations of the object.
     * 
     * @param rawPosition
     * @param unitsPerRotation
     * @return rotations
     */
    public static double rawToRotation(int rawPosition, double unitsPerRotation) {
        double rotations = (rawPosition / unitsPerRotation);
        return rotations;
    }
    
    /**
     * Converts raw sensor position to rotations. Units are full rotations of the object.
     * 
     * @param rotations
     * @param unitsPerRotation
     * @return rawPosition
     */
    public static double rotationsToRaw(double rotations, double unitsPerRotation) {
        return rotations * unitsPerRotation;
    }

    /**
     * Converts rotations to physical distance. Units are the same as the radius.
     * @param rotations
     * @param effectiveDiam
     * @return distance
     */
    public static double rotationsToDistance(double rotations, double effectiveDiam){
        double circumference = Math.PI * effectiveDiam;
        double distance = circumference * rotations;
        return distance;
    }

    /**
     * Converts distance to rotations. Units are the same as the radius.
     * @param distance
     * @param effectiveDiam
     * @return rotations
     */
    public static double distanceToRotations(double distance, double effectiveDiam){
        double circumference = Math.PI * effectiveDiam;
        double rotations = distance / circumference;
        return rotations;
    }

    /** Converts a raw position (delta or absolute) to a distance.
     * @param rawPosition
     * @param unitsPerRotation
     * @param effectiveDiam
     * @return distance
     * 
     */
    public static double rawToDistance(int rawPosition, double unitsPerRotation, double effectiveDiam) {
        return rotationsToDistance((rawToRotation(rawPosition, unitsPerRotation)), effectiveDiam);
    }

    /** Converts a distance to raw position (delta or absolute)
     * @param distance
     * @param unitsPerRotation
     * @param effectiveDiam
     * @return rawPosition
     * 
     */
    public static double distanceToRaw(double distance, double unitsPerRotation, double effectiveDiam) {
        return rotationsToRaw(distanceToRotations(distance, effectiveDiam), unitsPerRotation);
    }

    /**
     * Converts raw distance to degrees
     * @param rawPosition
     * @param unitsPerRotation
     * @return degrees
     */
    public static double rawToDegrees(int rawPosition, double unitsPerRotation) {
        return rawToRotation(rawPosition, unitsPerRotation) * 360;
    }

    /**
     * Converts degrees to raw distance
     * @param degrees
     * @param unitsPerRotation
     * @return rawPosition
     */
    public static double degreesToRaw(double degrees, double unitsPerRotation) {
        return rotationsToRaw(degrees / 360, unitsPerRotation);
    }


    @Override
    public void initDefaultCommand() {
      // Set the default command for a subsystem here.
      // setDefaultCommand(new MySpecialCommand());
    }
}