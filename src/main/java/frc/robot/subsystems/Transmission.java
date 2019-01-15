package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Transmission {
  private TalonSRX mMaster;

  private TalonSRX mSlave;

  public Transmission(int masterPort, int slavePort) {
    mMaster = new TalonSRX(masterPort);
    mSlave = new TalonSRX(slavePort);
  }
}