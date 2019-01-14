import static org.junit.Assert.*;

import org.junit.jupiter.api.Test;

import frc.robot.lib.TerriblePID;

public class PIDTests {

  @Test
  public void testBasicPID() {
    TerriblePID mPid = new TerriblePID(1, 1);
    TerriblePID mSecondaryPid = new TerriblePID(2, 2);

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
    // TerriblePID mIntegralPid = new TerriblePID(0, 1, -1, 1, 100000, 1);
    // mIntegralPid.setSetpoint(10);
    
    // double mOutput = mIntegralPid.update(9);
    // assertEquals(0.02, mOutput, 0.01);
  }

}