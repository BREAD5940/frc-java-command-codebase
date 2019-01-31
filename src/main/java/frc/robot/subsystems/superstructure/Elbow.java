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

  public double pidOutput = 0;

  public final double kTolerence = 1; // degrees
  public RotatingArmPeriodicIO mPeriodicIO = new RotatingArmPeriodicIO();
  
  public Elbow() {
    // pass the PID stuff, talon ports and mag encoder angle
    super(/* Name */ "Elbow", 
        new PIDSettings(/* kp */ 10d, /* ki */ 0d, /* kd */ 0d, /* kf */ 0d, FeedbackMode.ANGULAR),
        Arrays.asList(37),
        FeedbackDevice.CTRE_MagEncoder_Relative    
    );
    setAbsoluteTolerance(kTolerence);
  }

  @Override
  protected double returnPIDInput() {
    // return the location of the arm in degrees
    // return Math.toDegrees(getMaster().getSensorPosition().getValue());
    return Math.random() * 20;
  }

  public void setSetpoint(Rotation2d setpoint) {
    setSetpoint( setpoint.getDegree() );
  }

  @Override
  protected void usePIDOutput(double output) {
    System.out.println("elbow pid used");
    mPeriodicIO.pidOutput = output;
    pidOutput = output;
  }

  @Override
  protected void initDefaultCommand() {
  }
}