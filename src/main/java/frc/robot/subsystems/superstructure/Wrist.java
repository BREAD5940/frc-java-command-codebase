package frc.robot.subsystems.superstructure;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import org.ghrobotics.lib.mathematics.units.Rotation2d;

import frc.robot.lib.AbstractRotatingArm;
import frc.robot.lib.PIDSettings;
import frc.robot.lib.PIDSettings.FeedbackMode;


/**
 * A wrist subsystem with Talon hardware PID
 * Was never tested, the 775s burned out
 * 
 * @author Matthew Morley
 */
public class Wrist extends AbstractRotatingArm {

  public final double kTolerence = 5; // degrees
  
  public Wrist() {
    // pass the PID stuff, talon ports and mag encoder angle
    super(/* Name */ "Wrist", 
        new PIDSettings(/* kp */ 10d, /* ki */ 0d, /* kd */ 0d, /* kf */ 0d, FeedbackMode.ANGULAR),
        Arrays.asList(40),
        FeedbackDevice.CTRE_MagEncoder_Relative    
    );
    // setAbsoluteTolerance(kTolerence);
  }

  @Override
  protected void initDefaultCommand() {
  }



}