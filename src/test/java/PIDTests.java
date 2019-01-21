import static org.junit.Assert.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.lib.TerriblePID;

public class PIDTests {

  @Test
  public void testBasicPID() {

    // public TerriblePID(double kp, double ki, double kd, double kf, double minOutput, double maxOutput, 
    // double integralZone, double maxIntegralAccum, double rampRate, FeedForwardMode forwardMode,
    // FeedForwardBehavior feedforwardbehavior) {

    TerriblePID mPid = new TerriblePID(1 /* kp */, 0 /* ki */, 0 /* kd */, 0 /* kf */, -1 /* min output */, 1  /* max output */, 0  /* integral zone */, 1000  /* max integral accum */, 0 /* ramp rate */, null, null);
    TerriblePID mSecondaryPid = new TerriblePID(2 /* kp */, 0 /* ki */, 0 /* kd */, 0 /* kf */, -2 /* min output */, 2 /* max output */, 0  /* integral zone */, 1000  /* max integral accum */, 0 /* ramp rate */, null, null);

    mPid.setSetpoint(10);
    mSecondaryPid.setSetpoint(5);

    double mOutput = mPid.update(1);
    double mError = mPid.getError();
    double mSecondaryOutput = mSecondaryPid.update(3);
    double mSecondaryError = mSecondaryPid.getError();

    assertEquals(10, mPid.getSetpoint(), 0.01);
    assertEquals(9, mPid.getError(), 0.01);
    assertEquals(1, mPid.getOutput() , 0.01);

    assertEquals(5, mSecondaryPid.getSetpoint(), 0.01);
    assertEquals(2, mSecondaryPid.getError(), 0.01);
    assertEquals(2, mSecondaryPid.getOutput() , 0.01);
  }

  @Test
  public void testIntegral() {
    TerriblePID mIntegralPid = new TerriblePID(0, 1, 0, 0, -1, 1, 0, 1000, 0, null, null);
    mIntegralPid.setSetpoint(10);
    
    double mOutput = mIntegralPid.update(9);
    System.out.println("mOutput: " + mOutput);
    assertEquals(0.02, mOutput, 0.01);

    mOutput = mIntegralPid.update(9.5);
    System.out.println("mOutput: " + mOutput);
    assertEquals(0.03, mOutput, 0.01);
  }

  @Test
  public void testDerivative() {
    print("----- derivative pid test -----");
    TerriblePID mDerivativePid = new TerriblePID(0, 0, 1, 0, -1, 1, 0, 0, 0, null, null);
    mDerivativePid.setSetpoint(10);
    double mOutput = mDerivativePid.update(10);
    System.out.println("mOutput: " + mOutput);

    print(mDerivativePid.toString());
  }

  private void print(String string) {
    System.out.println(string);
  }

}