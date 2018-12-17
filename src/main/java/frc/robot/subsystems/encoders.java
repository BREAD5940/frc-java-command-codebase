
package frc.robot.subsystems;
import edu.wpi.first.wpilibj.command.Subsystem;

public class encoders extends Subsystem {
    public encoders() {
        super("encoders");
    }

    /**
     * Converts encoder native units to/from useful ones
     */
    public class conversion {
        public double encoderToMeasurement(double diameter, double pulsesPerRotation){
            return pulsesPerRotation; // TODO native unit conversion expansion
        }
    }




    @Override
    public void initDefaultCommand() {
      // Set the default command for a subsystem here.
      // setDefaultCommand(new MySpecialCommand());
    }
}