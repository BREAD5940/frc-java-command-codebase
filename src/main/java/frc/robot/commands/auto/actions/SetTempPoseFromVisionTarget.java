
package frc.robot.commands.auto.actions;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.commands.auto.groups.VisionCommandGroup;
import frc.robot.commands.auto.groups.VisionCommandGroup.PoseStorage;
import frc.robot.subsystems.LimeLight;

/**
 * Add your docs here.
 */
public class SetTempPoseFromVisionTarget extends InstantCommand {
	final double limelightOffsetFromFront;
	final VisionCommandGroup command;
	final boolean limelightFlipped;
	final PoseStorage toSet_;

	/**
	 * Set the temporary pose storage of a VisionCommandGroup. The pose set is vision target centric, NOT field centric!
	 * 
	 * @param command the command to interact with
	 * @param toSet the pose storage we should set
	 * @param limelightFlipped if the limelight is flipped up or not
	 */
	public SetTempPoseFromVisionTarget(VisionCommandGroup command, PoseStorage toSet, boolean limelightFlipped) {
		super();

		this.toSet_ = toSet;

		this.limelightOffsetFromFront = (limelightFlipped) ? 18 : 15; // TODO check numbers

		this.command = command;

		this.limelightFlipped = limelightFlipped;

		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called once when the command executes
	@Override
	public void initialize() {
		command.setPoseStorage(toSet_, LimeLight.getInstance().getPose(0));
	}

}
