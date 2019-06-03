package frc.robot.commands.subsystems.drivetrain;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;

import frc.robot.commands.auto.routines.AutoRoutine;

/**
 * Contains getters and settetrs in adition to local variables for tracking robot pose and stuff.
 * Because a b s t r a c t i o n I guess.
 * 
 * @author Matthew Morley
 */
public class VisionRoutine extends AutoRoutine {

	public VisionRoutine() {
		super();
		mPoseStorage1 = new Pose2d();
		mPoseStorage2 = mPoseStorage1;
	}

	private Pose2d mPoseStorage1, mPoseStorage2;
	private boolean pose1set = false;
	private boolean pose2set = false;

	/**
	 * @return the mPoseStorage1
	 */
	public Pose2d getPoseStorage1() {
		return mPoseStorage1;
	}

	/**
	 * The two different pose storages that auto command group stores
	 */
	public enum PoseStorage {
		POSE1, POSE2;
	}

	/**
	 * Set a pose storage from a requested slot and a value
	 * @param toSet
	 * @param value
	 */
	public void setPoseStorage(PoseStorage toSet, Pose2d value) {
		switch (toSet) {
		case POSE1:
			setPoseStorage1(value);
			break;
		case POSE2:
			setPoseStorage2(value);
			break;
		}
	}

	/**
	 * @param PoseStorage1 the `PoseStorage1 to set
	 */
	public void setPoseStorage1(Pose2d mPoseStorage1) {
		this.mPoseStorage1 = mPoseStorage1;
		this.pose1set = true;
	}

	/**
	 * @return the mPoseStorage2
	 */
	public Pose2d getPoseStorage2() {
		return mPoseStorage2;
	}

	/**
	 * @param mPoseStorage2 the mPoseStorage2 to set
	 */
	public void setPoseStorage2(Pose2d mPoseStorage2) {
		this.mPoseStorage2 = mPoseStorage2;
		this.pose2set = true;
	}

	/**
	 * @return the mPoseStorage1
	 */
	public Pose2d getmPoseStorage1() {
		return mPoseStorage1;
	}

	/**
	 * @param mPoseStorage1 the mPoseStorage1 to set
	 */
	public void setmPoseStorage1(Pose2d mPoseStorage1) {
		this.mPoseStorage1 = mPoseStorage1;
	}

	/**
	 * @return the mPoseStorage2
	 */
	public Pose2d getmPoseStorage2() {
		return mPoseStorage2;
	}

	/**
	 * @param mPoseStorage2 the mPoseStorage2 to set
	 */
	public void setmPoseStorage2(Pose2d mPoseStorage2) {
		this.mPoseStorage2 = mPoseStorage2;
	}

	/**
	 * @return the pose1set
	 */
	public boolean isPose1set() {
		return pose1set;
	}

	/**
	 * @param pose1set the pose1set to set
	 */
	public void setPose1set(boolean pose1set) {
		this.pose1set = pose1set;
	}

	/**
	 * @return the pose2set
	 */
	public boolean isPose2set() {
		return pose2set;
	}

	/**
	 * @param pose2set the pose2set to set
	 */
	public void setPose2set(boolean pose2set) {
		this.pose2set = pose2set;
	}

}
