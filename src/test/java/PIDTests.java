import static org.junit.Assert.assertEquals;

import org.junit.jupiter.api.Test;

import frc.robot.lib.TerriblePID;

public class PIDTests {

  @Test
  public void testBasicPID() {
    TerriblePID mPid = new TerriblePID(1, 0, -1, 1, 0, 1000);

    mPid.setSetpoint(10);

    double mOutput = mPid.update(0);
    double mError = mPid.getError();
    System.out.println("mOutput: " + mOutput + " mError: " + mError);

    assertEquals(10, mPid.getSetpoint(), 0.1);
  }

}