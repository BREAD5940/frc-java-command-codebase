package frc.robot.subsystems.superstructure;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.robot.lib.AbstractRotatingArm;
import frc.robot.lib.PIDSettings;
import frc.robot.lib.PIDSettings.FeedbackMode;


/**
 * A wrist subsystem with Talon hardware PID
 * Was never tested, the 775s burned out
 * 
 * @author Matthew Morley
 */
public class Elbow extends AbstractRotatingArm {

  public double pidOutput = 0;

  public final double kTolerence = 1; // degrees
  
  public Elbow() {
    // pass the PID stuff, talon ports and mag encoder angle
    super(/* Name */ "Elbow", 
        new PIDSettings(/* kp */ 10d, /* ki */ 0d, /* kd */ 0d, /* kf */ 0d, FeedbackMode.ANGULAR),
        Arrays.asList(37),
        FeedbackDevice.CTRE_MagEncoder_Relative    
    );
  }

  @Override
  protected void initDefaultCommand() {
  }

}