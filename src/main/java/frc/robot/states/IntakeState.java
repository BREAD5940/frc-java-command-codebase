package frc.robot.states;

public class IntakeState {

  public enum ClampState {
    OPEN,
    CLOSED;
  }

  public ClampState clampState = ClampState.CLOSED;
  public double intakeMotor = 0;
  public double wristAngle = 0;
  public double wristSetpoint = 0;
  public boolean hatchSensorTriggered = false;
  public boolean cargoSensorTriggered = false;

  public void setPower(double power) {
    intakeMotor = power;
  }


}