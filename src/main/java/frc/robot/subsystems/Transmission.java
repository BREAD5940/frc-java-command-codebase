package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Transmission {
  public static enum EncoderMode {
    NONE, CTRE_MagEncoder_Relative;
  }

  private TalonSRX mMaster;

  private TalonSRX mSlave;

  public Transmission(int masterPort, int slavePort, EncoderMode mode, boolean isInverted) {
    mMaster = new TalonSRX(masterPort);
    mSlave = new TalonSRX(slavePort);
    init(mode, isInverted);
  }

  private void init(EncoderMode mode, boolean isInverted) {
    mMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
    mSlave.set(ControlMode.Follower, mMaster.getDeviceID());
    mMaster.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, 30);
    // Quadrature Encoder of current
    // Talon
    mMaster.configPeakOutputForward(+1.0, 30);
    mMaster.configPeakOutputReverse(-1.0, 30);

    // mMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);

    if(isInverted) {
      mMaster.setInverted(true);
    }
  }
}