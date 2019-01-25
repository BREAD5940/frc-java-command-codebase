package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;
import org.ghrobotics.lib.wrappers.ctre.FalconSRX;
import org.ghrobotics.lib.wrappers.ctre.FalconSRXKt;

import frc.robot.lib.enums.TransmissionSide;

import frc.robot.RobotConfig.driveTrain;

public class Transmission {
  public static enum EncoderMode {
    NONE, CTRE_MagEncoder_Relative;
  }

  private FalconSRX mMaster;

  private FalconSRX mSlave;

  private TransmissionSide side;

  NativeUnitLengthModel lengthModel;

  public Transmission(int masterPort, int slavePort, EncoderMode mode, TransmissionSide side, boolean isInverted) {
    if (side == TransmissionSide.LEFT){ lengthModel = driveTrain.LEFT_NATIVE_UNIT_LENGTH_MODEL; } else { lengthModel = driveTrain.RIGHT_NATIVE_UNIT_LENGTH_MODEL; }

    mMaster = new FalconSRX(masterPort);
    mSlave = new FalconSRX(slavePort, lengthModel);

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

  


}