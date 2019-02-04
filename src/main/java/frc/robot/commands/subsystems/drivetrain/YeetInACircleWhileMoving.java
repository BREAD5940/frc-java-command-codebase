package frc.robot.commands.subsystems.drivetrain;

import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.lib.PIDSettings;
import frc.robot.lib.PIDSettings.FeedbackMode;

public class YeetInACircleWhileMoving extends PIDCommand {
  private final double target;
  double currentAngle;

  private static final PIDSettings kDefaultPIDGains = new PIDSettings(0.1, 0, 1, 0, FeedbackMode.ANGULAR);

  private static final double kDefaultPeriod = 0.01d;

  private static final double maxTurnSpeed = 0.5;

  private static final double angleTolerence = 10; // Degrees

  private final int direction;

  /**
   * Turn to a target absolute angle whole yeeting in a circle 
   * @param targetAbsoluteAngle
   */
  public YeetInACircleWhileMoving(double targetAbsoluteAngle, PIDSettings settings, double period, boolean isCloclwise) {
    super(settings.kp, settings.ki, settings.kd, period);
    requires(Robot.drivetrain);
    target = targetAbsoluteAngle;
    direction = (isCloclwise) ? 1 : -1;
    setSetpoint(target);
    setInputRange(-maxTurnSpeed, maxTurnSpeed);
    
    SmartDashboard.putData(this); // TODO does this break evertyhing
  }

  public YeetInACircleWhileMoving(double targetAbsoluteAngle) {
    this(targetAbsoluteAngle, kDefaultPIDGains, kDefaultPeriod, true);
  }

  @Override
  protected double returnPIDInput() {
    return Robot.drivetrain.getGyro();
  }

  @Override
  protected void usePIDOutput(double output) {
    Robot.drivetrain.setPowers(output * direction, -output * direction);
  }

  @Override
  protected void initialize() {
  }

  @Override
  protected void execute() {
  }

  @Override
  protected boolean isFinished() {
    return ( Math.abs(getPIDController().getError()) < angleTolerence 
        || (direction == 1 /* is clockwise */ && getPosition() > getSetpoint()) 
        || (direction == -1 /* is clockwise */ && getPosition() < getSetpoint()) 
    );
  }

  @Override
  protected void end() {
  }

  @Override
  protected void interrupted() {
  }
}
