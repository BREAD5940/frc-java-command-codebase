package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorTerm;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;
import org.ghrobotics.lib.wrappers.ctre.FalconSRX;

import frc.robot.RobotConfig.driveTrain;
import frc.robot.lib.enums.TransmissionSide;

public class Transmission {
  public static enum EncoderMode {
    NONE, CTRE_MagEncoder_Relative;
  }

  private FalconSRX<Length> mMaster;

  private FalconSRX<Length> mSlave;

  private TransmissionSide side;

  NativeUnitLengthModel lengthModel = driveTrain.LEFT_NATIVE_UNIT_LENGTH_MODEL;

  public Transmission(int masterPort, int slavePort, EncoderMode mode, TransmissionSide side, boolean isInverted) {
    if (side == TransmissionSide.LEFT){ lengthModel = driveTrain.LEFT_NATIVE_UNIT_LENGTH_MODEL; } else { lengthModel = driveTrain.RIGHT_NATIVE_UNIT_LENGTH_MODEL; }

    mMaster = new FalconSRX<Length>(masterPort, lengthModel, TimeUnitsKt.getMillisecond(10));
    mSlave = new FalconSRX<Length>(slavePort, lengthModel, TimeUnitsKt.getMillisecond(10));

    this.side = side;
    
    if(mode == EncoderMode.CTRE_MagEncoder_Relative) {
      mMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
      mMaster.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, 30);
    }
    mSlave.set(ControlMode.Follower, mMaster.getDeviceID());
    // Quadrature Encoder of current
    // Talon
    mMaster.configPeakOutputForward(+1.0, 30);
    mMaster.configPeakOutputReverse(-1.0, 30);

    if(isInverted) {
      mMaster.setInverted(true);
    }
  }

  public FalconSRX<Length> getMaster() {
    return mMaster;
  }

  public List<FalconSRX<Length>> getAll() {
    return Arrays.asList(
      mMaster, mSlave
    );
  }

  public NativeUnitLengthModel getModel() {
    return lengthModel;
  }

  public Length getDistance() {
    return mMaster.getSensorPosition();
  }

  public Velocity<Length> getVelocity() {
    return mMaster.getSensorVelocity();
  }

  public double getFeetPerSecond() {
    return VelocityKt.getFeetPerSecond(getVelocity());
  }

  public double getFeet() {
    return getDistance().getFeet();
  }

  public Length getClosedLoopError() {
    return lengthModel.toModel(NativeUnitKt.getSTU(mMaster.getClosedLoopError()));
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

}