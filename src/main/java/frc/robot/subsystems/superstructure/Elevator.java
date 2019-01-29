package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.command.Subsystem;

import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.commands.subsystems.superstructure.elevator.ElevatorTelop;
import frc.robot.lib.EncoderLib;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;


/**
 * The elevator subsystem controls the elevator height
 * with talon hardware PID. Contains methods for converting
 * from encoder units to height, and vice versa too!
 * 
 * @author Matthew Morley
 */
public class Elevator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public TalonSRX elevator_talon = new TalonSRX(RobotConfig.elevator.elevatorTalon.elevator_talon_port);

  double raw_max_height = EncoderLib.distanceToRaw(
    RobotConfig.elevator.elevator_maximum_height,
    RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION,
    RobotConfig.elevator.elevator_effective_diameter
  );
  float position_setpoint;
  public enum ElevatorPresets {
    LOW_ROCKET_PORT(27),
    MIDDLE_ROCKET_PORT(55),
    HIGH_ROCKET_PORT(84),
    LOW_ROCKET_HATCH(19),
    MIDDLE_ROCKET_HATCH(47),
    HIGH_ROCKET_HATCH(75),

    CARGO_SHIP_HATCH(20),
    // TODO this should be even with the low rocket hatch. According to the game manual, it isn't
    CARGO_SHIP_WALL(31);
    //top of wall

    private int height;

    ElevatorPresets(int height){
      this.height = height;
    }
    public int getValue(){
      return height;
    }
  }
  public static double getHeightEnumValue(ElevatorPresets height) {
    switch (height) {
      case LOW_ROCKET_PORT:
        return RobotConfig.auto.fieldPositions.low_rocket_port;
      case MIDDLE_ROCKET_PORT:
        return RobotConfig.auto.fieldPositions.middle_rocket_port;
      case HIGH_ROCKET_PORT:
        return RobotConfig.auto.fieldPositions.high_rocket_port;
      case LOW_ROCKET_HATCH:
        return RobotConfig.auto.fieldPositions.low_rocket_hatch;
      case MIDDLE_ROCKET_HATCH:
        return RobotConfig.auto.fieldPositions.middle_rocket_hatch;
      case HIGH_ROCKET_HATCH:
        return RobotConfig.auto.fieldPositions.high_rocket_hatch;
      case CARGO_SHIP_HATCH:
        return RobotConfig.auto.fieldPositions.cargo_ship_hatch;
      default:
        return 0;
    }
    // Reminder: Breaks aren't needed because of return
  }
  /** Set height to raise elevator to */
  public void init() {
    elevator_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,0,30);
    elevator_talon.setSelectedSensorPosition(0, 0, 10); // zero the encoder
    elevator_talon.setInverted(false);
    elevator_talon.setSensorPhase(true);
    this.elevator_talon.config_kP(0, RobotConfig.elevator.elevatorTalon.elevator_position_kp, 30);
    this.elevator_talon.config_kI(0, RobotConfig.elevator.elevatorTalon.elevator_position_ki, 30);
    this.elevator_talon.config_kD(0, RobotConfig.elevator.elevatorTalon.elevator_position_kd, 30);
    this.elevator_talon.config_kF(0, RobotConfig.elevator.elevatorTalon.elevator_position_kf, 30);
    setHeight(Robot.elevator.getElevatorAxisInches());
  }

  /**
   * Get the current elevator height from the throttle in inches. For passing into setHeight
   * @return height in inches
   */
  public double getElevatorAxisInches() {
    return (Robot.m_oi.getThrottleAxis() / (RobotConfig.controls.throttle_maximum_value - RobotConfig.controls.throttle_minimum_value)) 
      * RobotConfig.elevator.elevator_maximum_height;
  }
  /**
   * Set the elevator height, in inches
   * @param height in inches
   */
  public void setHeight(double height) {
    if (height>RobotConfig.elevator.elevator_maximum_height) {
      height = RobotConfig.elevator.elevator_maximum_height;//reset to maximum if too high
    } else if (height<RobotConfig.elevator.elevator_minimum_height) {
      height = RobotConfig.elevator.elevator_minimum_height;
    }
      //  however you may want to consider catching errors like elevator below minimum height, or elevator above maximum height, and allowing the elevator to move
    elevator_talon.set(
      ControlMode.Position, EncoderLib.distanceToRaw(
        height, 
        RobotConfig.elevator.elevator_effective_diameter, 
        RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION
      )
    );
  }

  public void setPercent(double percent){
    elevator_talon.set(ControlMode.PercentOutput,percent);
  }

  /**
   * Return the height of the elevator from zero, in inches
   * @return height in inches
   */
  public double getHeight() {
    double inches = EncoderLib.rawToDistance(
      elevator_talon.getSelectedSensorPosition(0), 
      RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION, 
      RobotConfig.elevator.elevator_effective_diameter);
    return inches;
    // return elevator_talon.getSelectedSensorPosition(0);
  }

  /**
   * Iteratively control the height of the elevator
   * a la an Xbox controller axis. Sets the height to current height
   * plus a delta.
   */
  public double iterativeSetHeight(double delta) {
    return delta + getHeight();
  }

  public boolean isWithinTolerence(double target) {
    return ( Math.abs(target - getHeight()) < RobotConfig.elevator.elevatorTolerences.position_tolerence );
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ElevatorTelop());
  }
}