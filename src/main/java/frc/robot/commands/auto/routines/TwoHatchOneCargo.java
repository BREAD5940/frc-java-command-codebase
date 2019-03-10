package frc.robot.commands.auto.routines;

import java.util.ArrayList;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import frc.robot.RobotConfig.auto.fieldPositions;
import frc.robot.commands.auto.AutoMotion;
import frc.robot.commands.auto.Trajectories;
import frc.robot.commands.auto.groups.AutoCommandGroup;
import frc.robot.commands.auto.groups.PickupHatch;
import frc.robot.commands.auto.groups.VisionCommandGroup;
import frc.robot.commands.subsystems.drivetrain.DriveDistanceTheSecond;
import frc.robot.commands.subsystems.drivetrain.DriveDistanceToVisionTarget;
import frc.robot.commands.subsystems.drivetrain.FollowVisionTargetTheSecond;
import frc.robot.commands.subsystems.superstructure.RunIntake;
import frc.robot.commands.subsystems.superstructure.SuperstructureGoToState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;
import frc.robot.subsystems.DriveTrain.TrajectoryTrackerMode;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.LimeLight.PipelinePreset;
import frc.robot.subsystems.superstructure.SuperStructure.iPosition;

/**
 * 2-hatch 1-cargo auto
 */
public class TwoHatchOneCargo extends VisionCommandGroup {
	// private AutoCommandGroup mBigCommandGroup;
	public ArrayList<TimedTrajectory<Pose2dWithCurvature>> trajects = new ArrayList<TimedTrajectory<Pose2dWithCurvature>>();
	public ArrayList<AutoMotion> motions = new ArrayList<AutoMotion>();

	public TwoHatchOneCargo(char arg1, char arg2) {
		this();
	}

	/**
	 * 2-hatch 1-cargo hard-coded auto. ow. This is fine. Everything is fine. 
	 * @param side to target (L or R)
	 * @param startPos L M or R on the hab
	 * @author Matthew Morley
	 */
	public TwoHatchOneCargo(/*char startPos, char side*/) {
		// HeldPiece cPiece = HeldPiece.HATCH; // we start with a hatch
		// String cStart = "hab" + startPos;

		boolean doIntake = false;
		boolean doVision = false;

		/* Get a trajectory to move to the cargo ship. THE ROBOT IS REVERSED */
		TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generatedLGTrajectories.get("habR" + " to " + "rocketRF"); //current trajectory from hashmap in Trajectories

		addParallel(new SuperstructureGoToState(iPosition.HATCH_SLAM_ROCKET_INSIDE_PREP)); // move arm inside to prep state
		addParallel(new LimeLight.SetLEDs(LimeLight.LEDMode.kON));
		addParallel(new LimeLight.setPipeline(PipelinePreset.k3dVision));
		addSequential(DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.LOW, true)); //drive to goal

		addParallel(new SuperstructureGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH));
		// addSequential(new SplineToVisionTarget(/*this.getPoseStorage1(), */LengthKt.getInch(0), LengthKt.getInch(30), 6.5));
		// addSequential(new FollowVisionTargetTheSecond(4.3));
		addSequential(new DriveDistanceToVisionTarget(LengthKt.getInch(35), 5));

		addSequential(new RunIntake(-1, 0, 1));

		// back up 3 feet
		addParallel(new SuperstructureGoToState(iPosition.HATCH_GRAB_INSIDE_PREP));
		addSequential(new DriveDistanceTheSecond(LengthKt.getFeet(3), true));

		// spline over to the rocket
		// var rocketToLoading = Trajectories.generatedLGTrajectories.get("rocketRF to loadingR");
		// addSequential(DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.HIGH, true)); //drive to goal

		addSequential(new PickupHatch());

		// var loadingToRocketClose = Trajectories.generatedLGTrajectories.get("loadingR to rocketRC");
		// addSequential(DriveTrain.getInstance().followTrajectoryWithGear(traject, TrajectoryTrackerMode.RAMSETE, Gear.HIGH, true)); //drive to goal
		addSequential(new LimeLight.SetLEDs(LimeLight.LEDMode.kON));

		addParallel(new SuperstructureGoToState(fieldPositions.hatchMiddleGoal, iPosition.HATCH));

		addSequential(new FollowVisionTargetTheSecond(5));

		addSequential(new RunIntake(-1, 0, 1));

		addParallel(new LimeLight.SetLEDs(LimeLight.LEDMode.kOFF));
		/* Move from middle of cargo ship to loading station on the same side to pick up a hatch */
		// cStart = "cargoM" + side;
		// cPiece = HeldPiece.NONE;

		// traject = Trajectories.generatedHGTrajectories.get("rocketLF" + " to " + "loading" + side); //current trajectory from hashmap in Trajectorie
		// this.addParallel(new SuperstructureGoToState(iPosition.HATCH_GRAB_INSIDE_PREP));
		// this.addSequential(DriveTrain.getInstance().followTrajectory(Trajectories.reverse3FeetLowGear, TrajectoryTrackerMode.RAMSETE, false)); //back up mega fast to clear rocket
		// this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal
		// addSequential(new SetTempPoseFromVisionTarget(this, PoseStorage.POSE1, false));
		// addSequential(new SplineToVisionTarget(this.getPoseStorage1(), LengthKt.getInch(0), LengthKt.getInch(30), 6.5));

		// /* Go right up to the cargo ship from the loading station */
		// cStart = "loading" + side;
		// cPiece = HeldPiece.HATCH;

		// traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "cargo" + side + '1'); //current trajectory from hashmap in Trajectorie
		// motion = new AutoMotion(GoalHeight.LOW, GoalType.CARGO_HATCH, false);
		// trajects.add(traject);
		// motions.add(motion);
		// this.addParallel(motion.getPrepCommand());
		// this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal

		// // turn 90 degrees to face the goal
		// this.addSequential(new TurnInPlace(Trajectories.locations.get("cargo" + side + '1').component2(), true)); // TODO check the angle math here! 
		// this.addSequential(motion.getBigCommandGroup()); //move the intake for hatch placement

		// /* Go from cargo side 1 to the depot */
		// cStart = "cargo" + side + '1';
		// cPiece = HeldPiece.NONE;

		// traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "depot" + side); //current trajectory from hashmap in Trajectorie
		// motion = new AutoMotion(GoalHeight.LOW, GoalType.RETRIEVE_CARGO, false);
		// trajects.add(traject);
		// motions.add(motion);
		// this.addParallel(motion.getPrepCommand());
		// this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal
		// this.addSequential(motion.getBigCommandGroup());
		// // this.addSequential(new DelayCommand(TimeUnitsKt.getSecond(0.5)).getWrappedValue());

		// /* Go from depot to cargo ship ~~2~~ 1 darnit you're right. Thanks 10pm me */
		// cStart = "depot" + side;
		// cPiece = HeldPiece.CARGO;
		// traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "cargo" + side + '1'); //current trajectory from hashmap in Trajectorie
		// motion = new AutoMotion(GoalHeight.OVER, GoalType.CARGO_CARGO, false);
		// trajects.add(traject);
		// motions.add(motion);
		// this.addParallel(motion.getPrepCommand());
		// this.addSequential(DriveTrain.getInstance().followTrajectory(traject, TrajectoryTrackerMode.RAMSETE, false)); //drive to goal
		// this.addSequential(new TurnInPlace(Rotation2dKt.getDegree(90), true)); // TODO check the angle
		// //FIXME this would have to raise the elevator
		// this.addSequential(motion.getBigCommandGroup()); //deposit cargo

	}

	// id functions

	/**
	 * identification function
	 * @return
	 *  the mBigCommandGroup of the function
	 */
	public AutoCommandGroup getBigCommandGroup() {
		return this;
	}

	//not id functions

}
