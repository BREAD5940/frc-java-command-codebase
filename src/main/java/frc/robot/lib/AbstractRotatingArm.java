package frc.robot.lib;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitRotationModel;
import org.ghrobotics.lib.wrappers.ctre.FalconSRX;

public abstract class AbstractRotatingArm extends LoopingSubsystem {

  private ArrayList<FalconSRX<Rotation2d>> motors = new ArrayList<FalconSRX<Rotation2d>>();
  
  private TerriblePID mPid;

  public RotatingArmPeriodicIO mPeriodicIO = new RotatingArmPeriodicIO();

  // private PIDSettings pidSettings;

  private NativeUnitRotationModel mRotationModel;

  // public AbstractRotatingArm(PIDSettings settings, int motorPort) {
  //   this(settings, motorPort, null, 0);
  // }

  /**
   * Create an abstract rotating arm using PIDSettings, a List of ports and a FeedbackDevice.
   * as of right now this only supports mag encoders. Remember to also have PID input and 
   * output methods, as this is a PIDSubsystem
   * @param PIDSettigns for the PIDSubsystem to use
   * @param motorPort on the CAN Bus (for single talon arms)
   * @param sensor for the arm to use (ONLY MAG ENCODER TO USE)
   */
  public AbstractRotatingArm(String name, PIDSettings settings, int motorPort, FeedbackDevice sensor) {
    this(name, settings, Arrays.asList(motorPort), sensor);
  }

  /**
   * Create an abstract rotating arm using PIDSettings, a List of ports and a FeedbackDevice.
   * as of right now this only supports mag encoders. Remember to also have PID input and 
   * output methods, as this is a PIDSubsystem
   * @param PIDSettigns for the PIDSubsystem to use
   * @param ports of talon CAN ports as a List
   * @param sensor for the arm to use (ONLY MAG ENCODER TO USE)
   */
  public AbstractRotatingArm(String name, PIDSettings settings, List<Integer> ports, FeedbackDevice sensor) {
    super(1);
    // super(name, settings.kp, settings.ki, settings.kd, settings.kf, 0.01f);
    mPid = new TerriblePID(settings.kp, settings.ki, settings.kd, 0, settings.minOutput, settings.maxOutput, settings.iZone, settings.maxIAccum, 10000, null, null);

    NativeUnit unitsPerRotation = NativeUnitKt.getSTU(0);

    // TODO add support for more sensors
    if(sensor == FeedbackDevice.CTRE_MagEncoder_Relative) {
      unitsPerRotation = NativeUnitKt.getSTU(4096);
    }

    mRotationModel = new NativeUnitRotationModel(unitsPerRotation);

    // add all of our talons to the list
    for( Integer i : ports ) {
      motors.add(new FalconSRX<Rotation2d>(i.intValue(), mRotationModel, TimeUnitsKt.getMillisecond(10)));
    }
    
    startLooper();
  }

  /**
   * Get the terriblePID instance
   */
  public TerriblePID getPid(){
    return mPid;
  }

  public void setSetpoint(double setpoint_) {
    mPid.setSetpoint(setpoint_);
  }

  public void setSetpoint(Rotation2d _setpoint) {
    setSetpoint(Math.toDegrees(_setpoint.getValue()) );
  }

  public Rotation2d getPosition() {
    return getMaster().getSensorPosition();
  }

  public double getSetpoint() {
    return mPid.getSetpoint();
  }

  /**
   * Get the master talon of the rotating arm
   */
  public FalconSRX<Rotation2d> getMaster() {
    return motors.get(0);
  }

  /**
   * Return an ArrayList of all the falconSRXes
   * @return motors... all of the motors
   */
  public ArrayList<FalconSRX<Rotation2d>> getAllMotors() {
    return motors;
  }

  /**
   * Get the Rotation2d of the encoder of the master talon
   * @return sensorPosition as a Rotation2d
   */
  public Rotation2d getRotation() {
    return getMaster().getSensorPosition();
  }

  /**
   * Set the position of the sensor to the given Rotation2d pos_
   * @param pos_ of the sensor as a Rotation2d
   */
  public void setRotation(Rotation2d pos_) {
    getMaster().setSensorPosition(pos_);
  }

  public static class RotatingArmPeriodicIO {
    public double setpoint = 0;
    public double feedForwardVoltage = 0;
    public double pidOutput = 0;
    public RotatingArmPeriodicIO() {
      feedForwardVoltage = 0;
      pidOutput = 0;
    }

    @Override
    public String toString() {
      return setpoint + ", " + feedForwardVoltage + ", " + pidOutput;
      // return "hellothere";
    }
  }

  public double getDegrees() {
    return Math.toDegrees(getMaster().getSensorPosition().getValue());
  }

  public void initilize() {}
  public void execute() {
    mPeriodicIO.pidOutput = mPid.update(getDegrees());
  }
  public void end() {}

  

}