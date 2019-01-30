package frc.robot.states;

/**
 * right now this is basically just a pair of doubles, but maybe
 * in the future it could also have different constants about the 
 * superstructure based on the angles
 * 
 * this all assumes that '0' is straight forwards on both joints
 */
public class IntakeAngle{

  private double elbowAngle;
  private double wristAngle;
  // TODO maybe have sanity checking on the angles to make sure they're not out of bounds in the context of the intake?

  public IntakeAngle(double eAngle, double wAngle){
    this.wristAngle=wAngle;
    this.elbowAngle=eAngle;
  }


  public double getMinHeight(){
    double min=0; //TODO remove instan.

    if(elbowAngle>0){
      min=0;
    }
    return min;
  }
}