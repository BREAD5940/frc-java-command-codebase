package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.lib.enums.TransmissionSide;

public class Transmission {
  public static enum EncoderMode {
    NONE, CTRE_MagEncoder_Relative;
  }

  private WPI_TalonSRX mMaster;

  private WPI_TalonSRX mSlave;

  private TransmissionSide side;

  public Transmission(int masterPort, int slavePort, EncoderMode mode, TransmissionSide side, boolean isInverted) {
    mMaster = new WPI_TalonSRX(masterPort);
    mSlave = new WPI_TalonSRX(slavePort);

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