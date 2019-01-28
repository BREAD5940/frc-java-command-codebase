package frc.robot.lib;

import edu.wpi.first.wpilibj.command.PIDSubsystem;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.wrappers.ctre.FalconSRX;

public abstract class AbstractPIDTransmission extends PIDSubsystem {

  private ArrayList<FalconSRX<Rotation2d>> motors = new ArrayList<FalconSRX<Rotation2d>>();

  private PIDSettings pidSettings;

  public AbstractPIDTransmission(PIDSettings settings, int motorPort) {
    this(settings, motorPort, null, 0);
  }

  public AbstractPIDTransmission(PIDSettings settings, int motorPort, FeedbackDevice sensor, double wheelRadius) {
    this(settings, motorPort, -1, sensor, wheelRadius);
  }

  public AbstractPIDTransmission(PIDSettings settings, int masterPort, int slavePort, FeedbackDevice sensor, double wheelRadius) {
    super(settings.kp, settings.ki, settings.kd, settings.kf, 0.01f);
  }

}