package frc.robot.commands.auto.routines;

import java.util.ArrayList;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.commands.auto.PrettyAutoMotion;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;

import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.auto.Trajectories;
import frc.robot.commands.auto.groups.VisionCommandGroup;
import frc.robot.commands.subsystems.superstructure.SuperstructureGoToState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.LimeLight.PipelinePreset;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

import org.ghrobotics.lib.wrappers.FalconMotor;

/**
 * 2-hatch 1-cargo auto
 */
public class LoadingToRocketF extends VisionCommandGroup {
	// private AutoCommandGroup mBigCommandGroup;
	public ArrayList<TimedTrajectory<Pose2dWithCurvature>> trajects = new ArrayList<TimedTrajectory<Pose2dWithCurvature>>();
	public ArrayList<PrettyAutoMotion> motions = new ArrayList<PrettyAutoMotion>();

	// public LoadingToRocketF(char arg1, char arg2) {
	// this();
	// }

	/**
	 * 2-hatch 1-cargo hard-coded auto. ow. This is fine. Everything is fine.
	 * 
	 * @param side     to target (L or R)
	 * @param startPos L M or R on the hab
	 * @author Matthew Morley
	 */
	public LoadingToRocketF(char startPos, char side) {
		// HeldPiece cPiece = HeldPiece.HATCH; // we start with a hatch
		// String cStart = "hab" + startPos;

		boolean doIntake = false;
		boolean doVision = false;

		/* Get a trajectory to move to the cargo ship. THE ROBOT IS REVERSED */
		TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generatedLGTrajectories.get(String.format("loading%s to rocket%sF", startPos, side)); //current trajectory from hashmap in Trajectories

		addCommands(

				// if (doIntake)
				/*addParallel*/(new LimeLight.SetLEDs(LimeLight.LEDMode.kON)),
				/*addParallel*/(new LimeLight.setPipeline(PipelinePreset.k3dVision)),
				/*addSequential*/(DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)).alongWith(
						/*addParallel*/(new SuperstructureGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH)) // move arm inside to prep state
				), //drive to goal
				/*addParallel*/(new SuperstructureGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH)));

		// );

	}

}
