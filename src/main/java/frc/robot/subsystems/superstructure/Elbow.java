package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.math.Util;
import frc.robot.RobotConfig;

import java.util.Arrays;
import java.util.Optional;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

import frc.robot.lib.AbstractRotatingArm;
import frc.robot.lib.EncoderLib;
import frc.robot.lib.PIDSettings;
import frc.robot.lib.PIDSettings.FeedbackMode;
import frc.robot.states.IntakeAngle;


/**
 * A wrist subsystem with Talon hardware PID
 * Was never tested, the 775s burned out
 * 
 * @author Matthew Morley
 */
public class Elbow extends AbstractRotatingArm {

  public final double kTolerence = 5; // degrees
  public RotatingArmPeriodicIO mPeriodicIO;
  
  public Elbow() {
    // pass the PID stuff, talon ports and mag encoder angle
    super(/* Name */ "Elbow", 
        new PIDSettings(/* kp */ 0.1, /* ki */ 0d, /* kd */ 0d, /* kf */ 0d, FeedbackMode.ANGULAR),
        Arrays.asList(37),
        FeedbackDevice.CTRE_MagEncoder_Relative    
    );
    setAbsoluteTolerance(kTolerence);
  }

  @Override
  protected double returnPIDInput() {
    // return the location of the arm in degrees
    return Math.toDegrees(getMaster().getSensorPosition().getValue());
  }

  public void setSetpoint(Rotation2d setpoint) {
    setSetpoint( setpoint.getDegree() );
  }

  @Override
  protected void usePIDOutput(double output) {
    mPeriodicIO.pidOutput = output;
  }

  @Override
  protected void initDefaultCommand() {
  }
}