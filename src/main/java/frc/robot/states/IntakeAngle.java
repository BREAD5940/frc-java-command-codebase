package frc.robot.states;

/**
 * right now this is basically just a pair of doubles, but maybe
 * in the future it could also have different constants about the 
 * superstructure based on the angles
 */
public class IntakeAngle{

  private double elbowAngle;
  private double wristAngle;

  public IntakeAngle(double eAngle, double wAngle){
    this.wristAngle=wAngle;
    this.elbowAngle=eAngle;
  }
}