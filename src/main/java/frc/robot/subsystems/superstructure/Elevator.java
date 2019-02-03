package frc.robot.subsystems.superstructure;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorTerm;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Time;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Acceleration;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;
import org.ghrobotics.lib.wrappers.ctre.FalconSRX;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotConfig;
import frc.robot.lib.obj.InvertSettings;
import frc.robot.states.ElevatorState;


/**
 * The elevator subsystem controls the elevator height
 * with talon hardware PID. Contains methods for converting
 * from encoder units to height, and vice versa too!
 * 
 * @author Matthew Morley
 */
public class Elevator /*extends Subsystem*/ {

    public static enum EncoderMode {
      NONE, CTRE_MagEncoder_Relative;
    }
  
    private FalconSRX<Length> mMaster;
  
    private FalconSRX<Length> mSlave1, mSlave2, mSlave3;
  
    // private TransmissionSide side;
  
    NativeUnitLengthModel lengthModel = RobotConfig.elevator.elevatorModel;
  
    public Elevator(int masterPort, int slavePort1, int slavePort2, int slavePort3, EncoderMode mode, InvertSettings settings) {
  
      mMaster = new FalconSRX<Length>(masterPort, lengthModel, TimeUnitsKt.getMillisecond(10));
      mSlave1 = new FalconSRX<Length>(slavePort1, lengthModel, TimeUnitsKt.getMillisecond(10));
      mSlave2 = new FalconSRX<Length>(slavePort2, lengthModel, TimeUnitsKt.getMillisecond(10));
      mSlave3 = new FalconSRX<Length>(slavePort3, lengthModel, TimeUnitsKt.getMillisecond(10));
      
      if(mode == EncoderMode.CTRE_MagEncoder_Relative) {
        mMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
        mMaster.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, 30);
      }
      
      mSlave1.set(ControlMode.Follower, mMaster.getDeviceID());
      mSlave2.set(ControlMode.Follower, mMaster.getDeviceID());
      mSlave3.set(ControlMode.Follower, mMaster.getDeviceID());


      // Quadrature Encoder of current
      // Talon
      mMaster.configPeakOutputForward(+1.0, 30);
      mMaster.configPeakOutputReverse(-1.0, 30);
  
      mMaster.setInverted(settings.masterInverted);
      mSlave1.setInverted(settings.slave1FollowerMode);
      mSlave2.setInverted(settings.slave1FollowerMode);
      mSlave3.setInverted(settings.slave1FollowerMode);
    }
  
    public FalconSRX<Length> getMaster() {
      return mMaster;
    }
  
    public List<FalconSRX<Length>> getAll() {
      return Arrays.asList(
        mMaster, mSlave1, mSlave2, mSlave3
      );
    }
  
    public NativeUnitLengthModel getModel() {
      return lengthModel;
    }
  
    public Length getHeight() {
      return mMaster.getSensorPosition();
    }
  
    public Velocity<Length> getVelocity() {
      return mMaster.getSensorVelocity();
    }
  
    public double getFeetPerSecond() {
      return VelocityKt.getFeetPerSecond(getVelocity());
    }
  
    public double getFeet() {
      return getHeight().getFeet();
    }
  
    public Length getClosedLoopError() {
      if (getMaster().getControlMode() != ControlMode.PercentOutput) {
        return lengthModel.toModel(NativeUnitKt.getSTU(mMaster.getClosedLoopError()));
      }
      else {
        return LengthKt.getFeet(0);
      }
    }
  
    public void zeroEncoder() {
      mMaster.setSensorPosition(LengthKt.getMeter(0));
    }
  
    public void setNeutralMode(NeutralMode mode) {
      for( FalconSRX<Length> motor : getAll() ) {
        motor.setNeutralMode(mode);
      }
    }
  
    public void stop() {
      getMaster().set(ControlMode.PercentOutput, 0);
    }
  
    public void setClosedLoopGains(double kp, double ki, double kd, double kf, double iZone, double maxIntegral) {
      mMaster.config_kP(0, kp, 30);
      mMaster.config_kI(0, ki, 30);
      mMaster.config_kD(0, kd, 30);
      mMaster.config_kF(0, kf, 30);
      mMaster.config_IntegralZone(0, (int)Math.round(lengthModel.fromModel(LengthKt.getMeter(iZone)).getValue()), 30);
      mMaster.configMaxIntegralAccumulator(0, maxIntegral, 0);
    }
    
  
  public ElevatorState getCurrentState(ElevatorState lastKnown) {
    Time time = TimeUnitsKt.getSecond(Timer.getFPGATimestamp());
    Acceleration<Length> accel = AccelerationKt.getAcceleration(LengthKt.getMeter(
          (getVelocity().getValue() - lastKnown.velocity.getValue())/(time.getValue() - lastKnown.time.getValue())));
    return new ElevatorState(getHeight(), getVelocity(), 
      accel, time);
  }
}
