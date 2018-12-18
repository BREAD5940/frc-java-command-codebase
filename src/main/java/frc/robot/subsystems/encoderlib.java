
package frc.robot.subsystems;
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
    public double rawToRotation(int rawPosition, double unitsPerRotation) {
        double rotations = rawPosition / unitsPerRotation;
        return rotations;
    }

    /**
     * Converts rotations to physical distance. Units are the same as the radius.
     * @param rotations
     * @param effectiveDiam
     * @return distance
     */
    public double positionToDistance(double rotations, double effectiveDiam){
        double circumference = Math.PI * effectiveDiam;
        double distance = circumference * rotations;
        return distance;
    }

    /** Converts a raw position (delta or absolute) to a distance.
     * @param rawPosition
     * @param unitsPerRotation
     * @param effectiveDiam
     * @return distance
     * 
     */
    public double rawToDistance(int rawPosition, double unitsPerRotation, double effectiveDiam) {
        return this.positionToDistance((this.rawToRotation(rawPosition, unitsPerRotation)), effectiveDiam);
    }


    @Override
    public void initDefaultCommand() {
      // Set the default command for a subsystem here.
      // setDefaultCommand(new MySpecialCommand());
    }
}