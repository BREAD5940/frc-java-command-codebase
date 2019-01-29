package frc.robot.lib;

import frc.robot.lib.TerriblePID.FeedForwardBehavior;
import frc.robot.lib.TerriblePID.FeedForwardMode;

public class PIDSettings {
  public double kp, ki, kd, kf, minOutput, maxOutput, iZone, maxIAccum;
  public FeedForwardMode feedForwardMode;
  public FeedForwardBehavior feedForwardBehavior;
  public enum feedbackMode {
    ANGULAR, LINEAR;
  }
}