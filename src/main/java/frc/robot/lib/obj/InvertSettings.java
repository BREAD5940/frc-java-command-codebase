package frc.robot.lib.obj;

import com.ctre.phoenix.motorcontrol.InvertType;

public class InvertSettings {
  boolean masterInverted;
  InvertType slave1FollowerMode, slave2FollowerMode, slave3FollowerMode;

  /**
   * Create an InvertSettings object. This can represent the state of a 4 motor transmission's motor inversion
   * @param mInvert if the master is inverted
   * @param s1FollowMode the mode of the first follower
   * @param s2FollowMode the mode of the second follower
   * @param s3FollowMode the mode of the third follower
   */
  public InvertSettings(boolean mInvert, InvertType s1FollowMode, InvertType s2FollowMode, InvertType s3FollowMode) {
    masterInverted = mInvert;
    slave1FollowerMode = s1FollowMode;
    slave2FollowerMode = s2FollowMode;
    slave3FollowerMode = s3FollowMode;
  }
}