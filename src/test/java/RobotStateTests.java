import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.Test;

import frc.math.Pose2d;
import frc.math.Rotation2d;
import frc.math.Translation2d;
import frc.robot.states.RobotState;

/**
 * Test the various 2d pose transforms and stuff used to track
 * the current state/pose of the robot.
 * @author Matthew Morley
 */
public class RobotStateTests {
  private RobotState mState = new RobotState();

  @Test
  public void testTranslationTolerences() {
    // print(String.format("Robot state: x{%s} y{%s}", mState.getPose().getTranslation().x(), mState.getPose().getTranslation().y()));
    Translation2d translate_ = new Translation2d(0.356, 0.18);
    boolean calculated_ = mState.isWithinTolerence(new Pose2d(translate_, new Rotation2d()) );  
    assertTrue(calculated_);
  }

  @Test
  public void testRobotTranslation() {
    Translation2d m_translate = new Translation2d(3, 4);
    


  }

  private void print(String stuff) {
    System.out.println(stuff);
  }
}