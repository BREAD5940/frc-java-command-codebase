package frc.robot.subsystems;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.team254.lib.physics.DCMotorTransmission;
import com.team254.lib.physics.DifferentialDrive;
import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotConfig;
import frc.robot.commands.subsystems.drivetrain.ArcadeDrive;
import frc.robot.commands.subsystems.drivetrain.TrajectoryTrackerCommand;

import org.ghrobotics.lib.localization.Localization;
import org.ghrobotics.lib.localization.TankEncoderLocalization;
import frc.robot.lib.EncoderLib;
import frc.robot.lib.Logger;
import frc.robot.lib.obj.DriveSignal;
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerDriveBase;

// import frc.robot.commands.drivetrain_shift_high;
// import frc.robot.commands.drivetrain_shift_low;

/**
 * Drivetrain subsystem. Initilizes the 4 drivetrain talons based on robotconfig
 * settings. Also, some motion profile stuff (which is hopefully depricated).
 * Contains methods for setting PID values, shifting, getting drivetrain encoder
 * status, setting speeds and voltages, and the arcade drive method.
 * 
 * @author Matthew Morley
 */
public class DriveTrain extends Subsystem {
  
  private static DriveTrain instance;

  public MotionProfileStatus m_left_MP_Status = new MotionProfileStatus();
  public MotionProfileStatus m_right_MP_Status = new MotionProfileStatus();

  private WPI_TalonSRX m_left_talon, s_left_talon, m_right_talon, s_right_talon;

  public AHRS gyro = new AHRS(SPI.Port.kMXP);
  double gyroZero;

  private Localization localization;

  public Localization getLocalization() {
    return localization;
  }

  private static double kQuickStopThreshold = 0.2;// DifferentialDrive.kDefaultQuickStopThreshold;
  private static double kQuickStopAlpha = 0.1;//DifferentialDrive.kDefaultQuickStopAlpha;
  private double quickStopAccumulator = 0;

  private RamseteTracker ramseteTracker;

  public enum Gear {
    LOW, HIGH;
  }
  Gear gear;

  private DCMotorTransmission transmission;
  private DifferentialDrive differentialDrive;


  private DriveTrain() {
    m_left_talon = new WPI_TalonSRX(RobotConfig.driveTrain.leftTalons.m_left_talon_port);
    s_left_talon = new WPI_TalonSRX(RobotConfig.driveTrain.leftTalons.s_left_talon_port);
    m_right_talon = new WPI_TalonSRX(RobotConfig.driveTrain.rightTalons.m_right_talon_port);
    s_right_talon = new WPI_TalonSRX(RobotConfig.driveTrain.rightTalons.s_right_talon_port);

    m_left_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
    m_right_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
    s_left_talon.set(ControlMode.Follower, m_left_talon.getDeviceID());
    s_right_talon.set(ControlMode.Follower, m_right_talon.getDeviceID());
    m_right_talon.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, 30);
    // Quadrature Encoder of current
    // Talon
    m_left_talon.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, 30);
    // Quadrature Encoder of current
    // Talon
    m_left_talon.configPeakOutputForward(+1.0, 30);
    m_left_talon.configPeakOutputReverse(-1.0, 30);
    m_right_talon.configPeakOutputForward(+1.0, 30);
    m_right_talon.configPeakOutputReverse(-1.0, 30);

    m_left_talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);
    m_right_talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);

    m_left_talon.setInverted(true);
    s_left_talon.setInverted(true);


    localization = new TankEncoderLocalization(
      () -> Rotation2dKt.getDegree(DriveTrain.getInstance().getGyro()),
      () -> LengthKt.getFeet(DriveTrain.getInstance().getLeftDistance() ),
      () -> LengthKt.getFeet(DriveTrain.getInstance().getRightDistance())
    );

    transmission = new DCMotorTransmission(
      1 / Constants.kVDrive,
      Constants.kWheelRadius * Constants.kWheelRadius * Constants.kRobotMass / (2.0 * Constants.kADrive),
      Constants.kStaticFrictionVoltage
    );

    differentialDrive = new DifferentialDrive(
      Constants.kRobotMass,
      Constants.kRobotMomentOfInertia,
      Constants.kRobotAngularDrag,
      Constants.kWheelRadius,
      Constants.kTrackWidth / 2.0,
      transmission,
      transmission
    );

    ramseteTracker = new RamseteTracker(Constants.kDriveBeta, Constants.kDriveZeta);

  }

  public synchronized static DriveTrain getInstance() {
    if (instance == null) instance = new DriveTrain();
    return instance;
  }

  @Override
  public WPI_TalonSRX getLeftMotor() {
    return m_left_talon;
  }
  
  public RamseteTracker getRamseteTracker() {
    return ramseteTracker;
  }

  public void init() {
    zeroEncoders();
    setHighGear();
  }

  public void setHighGear() {
    this.m_left_talon.config_kP(0, RobotConfig.driveTrain.leftTalons.velocity_kp_high, 30);
    this.m_left_talon.config_kI(0, RobotConfig.driveTrain.leftTalons.velocity_ki_high, 30);
    this.m_left_talon.config_kD(0, RobotConfig.driveTrain.leftTalons.velocity_kd_high, 30);
    this.m_left_talon.config_kF(0, RobotConfig.driveTrain.leftTalons.velocity_kf_high, 30);
    this.m_left_talon.config_IntegralZone(0, RobotConfig.driveTrain.leftTalons.velocity_izone_high, 30);
    // this.m_left_talon.configMaxIntegralAccumulator(0,
    // RobotConfig.driveTrain.left_talons.velocity_max_integral_high, 0);

    this.m_right_talon.config_kP(0, RobotConfig.driveTrain.rightTalons.velocity_kp_high, 30);
    this.m_right_talon.config_kI(0, RobotConfig.driveTrain.leftTalons.velocity_ki_high, 30);
    this.m_right_talon.config_kD(0, RobotConfig.driveTrain.leftTalons.velocity_kd_high, 30);
    this.m_right_talon.config_kF(0, RobotConfig.driveTrain.leftTalons.velocity_kf_high, 30);
    this.m_right_talon.config_IntegralZone(0, RobotConfig.driveTrain.leftTalons.velocity_izone_high, 30);
    // this.m_right_talon.configMaxIntegralAccumulator(0,
    // RobotConfig.m_right_velocity_max_integral_high, 0);

    // Trigger solenoids
    Robot.drivetrain_shift_high();
    gear = Gear.HIGH;
  }

  public void setLowGear() {
    this.m_left_talon.config_kP(0, RobotConfig.driveTrain.leftTalons.velocity_kp_low, 0);
    this.m_left_talon.config_kI(0, RobotConfig.driveTrain.leftTalons.velocity_ki_low, 0);
    this.m_left_talon.config_kD(0, RobotConfig.driveTrain.leftTalons.velocity_kd_low, 0);
    this.m_left_talon.config_kF(0, RobotConfig.driveTrain.leftTalons.velocity_kf_low, 0);
    this.m_left_talon.config_IntegralZone(0, RobotConfig.driveTrain.leftTalons.velocity_izone_low, 0);
    // this.m_left_talon.configMaxIntegralAccumulator(0,
    // RobotConfig.driveTrain.left_talons.velocity_max_integral_low, 0);

    this.m_right_talon.config_kP(0, RobotConfig.driveTrain.rightTalons.velocity_kp_low, 0);
    this.m_right_talon.config_kI(0, RobotConfig.driveTrain.rightTalons.velocity_ki_low, 0);
    this.m_right_talon.config_kD(0, RobotConfig.driveTrain.rightTalons.velocity_kd_low, 0);
    this.m_right_talon.config_kF(0, RobotConfig.driveTrain.rightTalons.velocity_kf_low, 0);
    this.m_right_talon.config_IntegralZone(0, RobotConfig.driveTrain.rightTalons.velocity_izone_low, 0);
    // this.m_right_talon.configMaxIntegralAccumulator(0,
    // RobotConfig.m_right_velocity_max_integral_low, 0);

    // Trigger solenoids
    Robot.drivetrain_shift_low();

    gear = Gear.LOW;
  }

  public void setGear(Gear gear) {
    switch (gear) {
      case HIGH:
        setHighGear();
        break;
      case LOW:
        setLowGear();
        break;
      default:
        break;
    }
  }

  /** Return the current distance in feet of the left wheels */
  public double getLeftDistance() {
    return EncoderLib.rawToDistance(
      this.m_left_talon.getSelectedSensorPosition(0),
      RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION,
      RobotConfig.driveTrain.left_wheel_effective_diameter / 12
    );
  }

  public double getRightDistance() {
    return EncoderLib.rawToDistance(
      this.m_right_talon.getSelectedSensorPosition(0),
      RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION,
      RobotConfig.driveTrain.right_wheel_effective_diameter
    ) / 12;
  }

  public double getLeftVelocity() {
    return EncoderLib.rawToDistance(
      this.m_left_talon.getSelectedSensorVelocity(0) * 10,
      // Mulitply by 10 because units
      // are per 100ms
      RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION,
      RobotConfig.driveTrain.left_wheel_effective_diameter
    );
  }

  public double getRightVelocity() {
    return EncoderLib.rawToDistance(
      this.m_right_talon.getSelectedSensorVelocity(0) * 10,
      RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION,
      RobotConfig.driveTrain.right_wheel_effective_diameter
    );
  }

  public double getAverageVelocity() {
    return (getLeftVelocity() + getRightVelocity()) / 2;
  }

  public void zeroEncoders() {
    m_left_talon.setSelectedSensorPosition(0, 0, 30);
    m_right_talon.setSelectedSensorPosition(0, 0, 30);
  }

  /**
   * An even more lazy version of setSpeeds This will literally set the
   * voltage of the left and right talons (from -12 to 12 ofc per battery voltage)
   * 
   * @param left_voltage
   * @param right_voltage
   */
  public void setVoltages(double left_voltage, double right_voltage) {
    m_left_talon.set(ControlMode.PercentOutput, left_voltage / 12);
    m_right_talon.set(ControlMode.PercentOutput, right_voltage / 12);
  }

  /**
   * setPowers is an even more lazy version of set speeds. This will literally set the
   * throttle of the left and right talons (from -1 to 1 ofc, like normal)
   * 
   * @param left_power
   * @param right_power
   */
  public void setPowers(double left_power, double right_power) {
    m_left_talon.set(ControlMode.PercentOutput, left_power);
    m_right_talon.set(ControlMode.PercentOutput, right_power);
  }

  /**
   * Set the neutral mode of the talons
   * @param mode the mode to set the talons to
   */
  public void setMode(NeutralMode mode) {
    m_left_talon.setNeutralMode(mode);
    m_right_talon.setNeutralMode(mode);
    s_left_talon.setNeutralMode(mode);
    s_right_talon.setNeutralMode(mode);
}

  /**
   * Set the target left speed. Units are in raw units.
   * 
   * @param speed in raw units per 100ms
   */
  public void setLeftSpeedRaw(double speed) {
    m_left_talon.set(ControlMode.Velocity, speed);
  }

  /**
   * Set the target right speed. Units are in raw units.
   * 
   * @param speed in raw units per 100ms
   */
  public void setRightSpeedRaw(double speed) {
    m_right_talon.set(ControlMode.Velocity, speed);
  }

  /**
   * Set the velocity of the wheels using raw units per 100ms velocities, feedforward rates
   * and accelerations.
   * @param signal
   * @param feedforward
   * @param leftAccel
   * @param rightAccel
   */
  public void setVelocity(DriveSignal signal, DriveSignal feedforward, double leftAccel, double rightAccel) {


    //  m_left_talon.set(ControlMode.Velocity, signal.getLeft(), DemandType.ArbitraryFeedForward,
    //          feedforward.getLeft());// + Constants.kDriveLowGearVelocityKd * leftAccel / 1023.0);
    //  m_right_talon.set(ControlMode.Velocity, signal.getRight(), DemandType.ArbitraryFeedForward,
    //          feedforward.getRight());// + Constants.kDriveLowGearVelocityKd * rightAccel / 1023.0);

    m_left_talon.set(ControlMode.Velocity, signal.getLeft(), DemandType.ArbitraryFeedForward,
        feedforward.getLeft() + leftAccel * Constants.kDriveLowGearVelocityKa);

    m_right_talon.set(ControlMode.Velocity, signal.getRight(), DemandType.ArbitraryFeedForward,
            feedforward.getRight() + rightAccel * Constants.kDriveLowGearVelocityKa);

    // m_left_talon.set(ControlMode.Velocity, signal.getLeft());// DemandType.ArbitraryFeedForward,
    //         // feedforward.getLeft() + Constants.kDriveLowGearVelocityKd * leftAccel / 1023.0);
    // m_right_talon.set(ControlMode.Velocity, signal.getRight());//, DemandType.ArbitraryFeedForward,
    //         // feedforward.getRight() + Constants.kDriveLowGearVelocityKd * rightAccel / 1023.0);

    // setPowers(0.3, 0.3);

    // Logger.log(String.format("Set velocity: left speed %s left feedforward %s right speed %s right feedforward %s",
    //   signal.getLeft(), feedforward.getLeft() + Constants.kDriveLowGearVelocityKa * leftAccel / 1023.0,
    //   signal.getRight(), feedforward.getRight() + Constants.kDriveLowGearVelocityKd * rightAccel / 1023.0
    // ));

  }


  // public void arcadeDriveMethod(double forwardspeed, double turnspeed) {
  //   // double forwardspeed = Robot.m_oi.getForwardAxis() * -1;
  //   // double turnspeed = Robot.m_oi.getTurnAxis();

  //   // if ((forwardspeed < 0.02) && (forwardspeed > -0.02)) {
  //   //   forwardspeed = 0;
  //   // }
  //   // if ((turnspeed < 0.01) && (turnspeed > -0.01)) {
  //   //   turnspeed = 0;
  //   // }

  //   // if (RobotConfig.controls.driving_squared) {
  //     forwardspeed = forwardspeed * Math.abs(forwardspeed);
  //     turnspeed = turnspeed * Math.abs(turnspeed);
  //   // }
  //   // if (Robot.drivetrain.gear == Gear.HIGH) {
  //   //   forwardspeed = forwardspeed * RobotConfig.driveTrain.max_forward_speed_high;
  //   //   turnspeed = turnspeed * RobotConfig.driveTrain.max_turn_speed_high;
  //   // }
  //   // if (Robot.drivetrain.gear == Gear.LOW) {
  //   //   forwardspeed = forwardspeed * RobotConfig.driveTrain.max_forward_speed_low;
  //   //   turnspeed = turnspeed * RobotConfig.driveTrain.max_turn_speed_low;
  //   // }

  //   SmartDashboard.putString("forwardspeed / turnspeed: ", forwardspeed + " / " + turnspeed);

  //   // turnspeed = turnspeed * 0.7;

  //   m_left_talon.set(ControlMode.PercentOutput, forwardspeed, DemandType.ArbitraryFeedForward, turnspeed );
  //   m_right_talon.set(ControlMode.PercentOutput, forwardspeed, DemandType.ArbitraryFeedForward, -turnspeed );
  //   // s_right_talon.set(ControlMode.PercentOutput, forwardspeed, DemandType.ArbitraryFeedForward, -turnspeed );

  //   double leftspeed = forwardspeed * 4 + turnspeed * 5; // units are in feet
  //   double rightspeed = forwardspeed * 4 - turnspeed * 5;

  //   setMode(NeutralMode.Brake);
  //   m_left_talon.configClosedloopRamp(0.2);
  //   m_right_talon.configClosedloopRamp(0.2);

  //   /**
  //    * Set left speed raw in feet per 100ms
  //    */
  //   // double leftspeedraw = EncoderLib.distanceToRaw(
  //   //   leftspeed,
  //   //   RobotConfig.driveTrain.left_wheel_effective_diameter / 12,
  //   //   RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION
  //   // ) / 10;
  //   // ((leftspeed) / (Math.PI *
  //   // RobotConfig.driveTrain.left_wheel_effective_diameter
  //   // / 12)) *
  //   // RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION
  //   // / 10;
  //   // divide by 10 becuase the talons want units per 100ms
  //   // double rightspeedraw = EncoderLib.distanceToRaw(
  //   //   rightspeed,
  //   //   RobotConfig.driveTrain.right_wheel_effective_diameter / 12,
  //   //   RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION
  //   // ) / 10;

  //   // setPowers(leftspeed, rightspeed);
  //   // setFeetPerSecond(leftspeed, rightspeed);

  //   // setPowers(0.5, 0.5);

  //   // setSpeeds(-500, -500);
  // }

  public Pose2d getRobotPosition(){
    return getLocalization().getRobotPosition();
  }

  public void setRobotPosition(Pose2d pose2d){
    getLocalization().reset(pose2d);
  }

  public void arcadeDrive(double linearPercent, double rotationPercent){
    double maxInput = Math.signum(Math.max(Math.abs(linearPercent), Math.abs(rotationPercent)));

    double leftMotorOutput, rightMotorOutput;

    if (linearPercent >= 0.0) {
        // First quadrant, else second quadrant
        if (rotationPercent >= 0.0) {
            leftMotorOutput = maxInput;
            rightMotorOutput = linearPercent - rotationPercent;
        } else {
            leftMotorOutput = linearPercent + rotationPercent;
            rightMotorOutput = maxInput;
        }
    } else {
        // Third quadrant, else fourth quadrant
        if (rotationPercent >= 0.0) {
            leftMotorOutput = linearPercent + rotationPercent;
            rightMotorOutput = maxInput;
        } else {
            leftMotorOutput = maxInput;
            rightMotorOutput = linearPercent - rotationPercent;
        }
    }

    tankDrive(leftMotorOutput, rightMotorOutput);
  }

  public void curvatureDrive(double linearPercent, double curvaturePercent, boolean isQuickTurn) {
      double angularPower;
      boolean overPower;

      if (isQuickTurn) {
          if (Math.abs(linearPercent) < kQuickStopThreshold) {
              quickStopAccumulator = (1 - kQuickStopAlpha) * quickStopAccumulator +
                      kQuickStopAlpha * RangesKt.coerceIn(curvaturePercent, -1, 1) * 2.0;
          }

          overPower = true;
          angularPower = curvaturePercent;
      } else {
          overPower = false;
          angularPower = Math.abs(linearPercent) * curvaturePercent - quickStopAccumulator;

          if(quickStopAccumulator > 1){
              quickStopAccumulator -= 1;
          }else if(quickStopAccumulator < -1){
              quickStopAccumulator += 1;
          }else{
              quickStopAccumulator = 0;
          }
      }

      double leftMotorOutput = linearPercent + angularPower;
      double rightMotorOutput = linearPercent - angularPower;

      // If rotation is overpowered, reduce both outputs to within acceptable range
      if (overPower) {
          if(leftMotorOutput > 1.0){
              rightMotorOutput -= leftMotorOutput - 1.0;
              leftMotorOutput = 1.0;
          }else if(rightMotorOutput > 1.0) {
              leftMotorOutput -= rightMotorOutput - 1.0;
              rightMotorOutput = 1.0;
          }else if(leftMotorOutput < -1.0) {
              rightMotorOutput -= leftMotorOutput + 1.0;
              leftMotorOutput = -1.0;
          }else if(rightMotorOutput < -1.0){
              leftMotorOutput -= rightMotorOutput + 1.0;
              rightMotorOutput = -1.0;
          }
      }

      // Normalize the wheel speeds
      double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
      if (maxMagnitude > 1.0) {
          leftMotorOutput /= maxMagnitude;
          rightMotorOutput /= maxMagnitude;
      }

      tankDrive(leftMotorOutput, rightMotorOutput);
  }

  public void tankDrive(double leftPercent, double rightPercent){
    m_left_talon.setPercentOutput(leftPercent);
    getRightMotor().setPercentOutput(rightPercent);
  }

  public TrajectoryTrackerCommand followTrajectory(TimedTrajectory<Pose2dWithCurvature> trajectory){
      return followTrajectory(trajectory, false);
  }

  public TrajectoryTrackerCommand followTrajectory(TimedTrajectory<Pose2dWithCurvature> trajectory, boolean reset){
      return new TrajectoryTrackerCommand(this, this, () -> trajectory, reset);
  }


  /**
   * Get the angle of the gyro, accounting for the gyro zero angle
   * @return compensated gyro angle
   */
  public double getGyro() {
    return gyro.getAngle() - gyroZero;
  }

  public void zeroGyro() {
    gyroZero = gyro.getAngle();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ArcadeDrive());
    // setDefaultCommand(new auto_action_DRIVE(5, "high", 5, 30));
  }
}