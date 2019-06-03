import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.junit.jupiter.api.Test;

import frc.robot.commands.auto.routines.yeOldeRoutines.Trajectories;
import frc.robot.lib.Logger;
import frc.robot.lib.motion.Util;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.lib.statemachines.AutoMotionStateMachine.HeldPiece;
import frc.robot.states.ElevatorState;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.RotatingJoint.RotatingArmState;

public class AutoPathingCSVGenerator {

	@Test
	public void twoHatchLLtest() {

		var side = 'r';

		boolean isLeft = (side == 'L' || side == 'l');

		var fallOFfHab = Arrays.asList(
				new Pose2d(LengthKt.getFeet(5.175),
						LengthKt.getFeet(17.689),
						Rotation2dKt.getDegree(0)),
				new Pose2d(LengthKt.getFeet(8),
						LengthKt.getFeet(18.5),
						Rotation2dKt.getDegree(40))

		);

		var floorToRocketF = Arrays.asList(
				new Pose2d(LengthKt.getFeet(8.0),
						LengthKt.getFeet(18.5),
						Rotation2dKt.getDegree(40)),
				new Pose2d(LengthKt.getFeet(15.14),
						LengthKt.getFeet(24.122),
						Rotation2dKt.getDegree(32)));

		var rocketCToLoading = Arrays.asList(
				new Pose2d(LengthKt.getFeet(15.7),
						LengthKt.getFeet(24.37),
						Rotation2dKt.getDegree(32)),
				new Pose2d(LengthKt.getFeet(5.572),
						LengthKt.getFeet(23.017),
						Rotation2dKt.getDegree(-10)),
				new Pose2d(LengthKt.getFeet(5),
						LengthKt.getFeet(24.918),
						Rotation2dKt.getDegree(180)),
				new Pose2d(LengthKt.getFeet(6.5),
						LengthKt.getFeet(24.882),
						Rotation2dKt.getDegree(180)));
		// Rotation2dKt.getDegree(180);

		var loadingToRocketF = Arrays.asList(
				new Pose2d(LengthKt.getFeet(1.465),
						LengthKt.getFeet(24.871),
						Rotation2dKt.getDegree(180)),
				new Pose2d(LengthKt.getFeet(19.871),
						LengthKt.getFeet(22.601),
						Rotation2dKt.getDegree(-154.855)),
				new Pose2d(LengthKt.getFeet(23.772),
						LengthKt.getFeet(23.551),
						Rotation2dKt.getDegree(148)));

		var rocketFtoLoading = Arrays.asList(
				new Pose2d(LengthKt.getFeet(22.438),
						LengthKt.getFeet(24.194),
						Rotation2dKt.getDegree(148)),
				new Pose2d(LengthKt.getFeet(21.338),
						LengthKt.getFeet(22.673),
						Rotation2dKt.getDegree(0)),
				new Pose2d(LengthKt.getFeet(6.023),
						LengthKt.getFeet(22.892),
						Rotation2dKt.getDegree(-4)),
				new Pose2d(LengthKt.getFeet(4.5),
						LengthKt.getFeet(24.8),
						Rotation2dKt.getDegree(180)));

		if (!isLeft) {
			fallOFfHab = Util.reflectTrajectory(fallOFfHab);
			floorToRocketF = Util.reflectTrajectory(floorToRocketF);
			rocketCToLoading = Util.reflectTrajectory(rocketCToLoading);
			loadingToRocketF = Util.reflectTrajectory(loadingToRocketF);
			rocketFtoLoading = Util.reflectTrajectory(rocketFtoLoading);
		}

		var t_fallOFfHab = Trajectories.generateTrajectory(fallOFfHab, Trajectories.kLowGearConstraints,

				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(5.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(6.0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				false,
				true);

		var t_floorToRocketF = Trajectories.generateTrajectory(floorToRocketF, Trajectories.kLowGearConstraints,

				VelocityKt.getVelocity(LengthKt.getFeet(5.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(3.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(6.0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				false,
				true);

		var t_rocketLFToLoading = Trajectories.generateTrajectory(rocketCToLoading, Trajectories.kLowGearConstraints,

				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(6.0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				true,
				true);

		var t_loadingToRocketF = Trajectories.generateTrajectory(loadingToRocketF, Trajectories.kLowGearConstraints,

				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(7.0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				true,
				true);

		var t_rocketFtoLoading = Trajectories.generateTrajectory(rocketFtoLoading, Trajectories.kLowGearConstraints,

				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(7.0)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				true,
				true);

		var path = trajectToArrayList(t_fallOFfHab);
		var path2 = trajectToArrayList(t_floorToRocketF);
		var path3 = trajectToArrayList(t_rocketLFToLoading);
		var path4 = trajectToArrayList(t_loadingToRocketF);
		var path5 = trajectToArrayList(t_rocketFtoLoading);

		path.addAll(path2);
		path.addAll(path3);
		path.addAll(path4);
		path.addAll(path5);

		writeToCSV("src/main/python/twoHatchLLtest.csv", path);

		// addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_fallOFfHab, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, true));

		// addParallel(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));
		// addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_floorToRocketF, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, false));

		// addSequential(new PlaceHatch());

		// addParallel(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE));
		// addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_rocketLFToLoading, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, false));
		// addSequential(new DriveDistanceToVisionTarget(LengthKt.getInch(20), VelocityKt.getVelocity(LengthKt.getFeet(3))));

		// addSequential(new GrabHatch());

		// addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_loadingToRocketF, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, true));
		// addSequential(new JankyGoToState(fieldPositions.hatchLowGoal, iPosition.HATCH));

		// addSequential(new PlaceHatch());

		// addParallel(new JankyGoToState(iPosition.HATCH_GRAB_INSIDE));
		// addSequential(DriveTrain.getInstance().followTrajectoryWithGear(t_rocketFtoLoading, TrajectoryTrackerMode.RAMSETE, DriveTrain.Gear.LOW, true));

		// }
		// }

	}

	public ArrayList<SuperStructureState> injectStates(ArrayList<SuperStructureState> wps, ArrayList<TimedTrajectory<Pose2dWithCurvature>> bigPath) {
		double elevatorInchPerSecond = 12;
		double elbowRadPerSecond = 3.14159;
		double wristRadPerSecond = 3.14159;
		ArrayList<SuperStructureState> toReturn = new ArrayList<SuperStructureState>();

		if (wps.size() != bigPath.size()) {
			System.out.println("ur bad");
			return toReturn;
		}

		toReturn.add(0, new SuperStructureState());
		for (int i = 0; i < bigPath.size(); i++) {
			SuperStructureState currentGoal = wps.get(i);
			for (int j = 0; j < bigPath.get(i).getPoints().size() - 1; j++) {
				if (j == 0) {
					toReturn.add(toReturn.get(toReturn.size() - 1));
				} else {
					TimedEntry<Pose2dWithCurvature> point = bigPath.get(i).getPoints().get(j);
					double deltaT = (point.getT().getSecond() - bigPath.get(i).getPoints().get(j - 1).getT().getSecond()) / 2;
					SuperStructureState prevState = toReturn.get(toReturn.size() - 1);

					double nextHeight = 0; //FIXME this requires math and ifs to make it move the right distance in the right direction
					double nextElbow = 0;
					double nextWrist = 0;

					if (currentGoal.getElevatorHeight().getInch() - prevState.getElevatorHeight().getInch() > 0) {
						nextHeight = prevState.getElevatorHeight().getInch() + elevatorInchPerSecond / deltaT;
					} else {
						nextHeight = prevState.getElevatorHeight().getInch() - elevatorInchPerSecond / deltaT;
					}

					if (currentGoal.getElbowAngle().getRadian() - prevState.getElbowAngle().getRadian() > 0) {
						nextElbow = prevState.getElbowAngle().getRadian() + elbowRadPerSecond / deltaT;
					} else {
						nextElbow = prevState.getElbowAngle().getRadian() - elbowRadPerSecond / deltaT;
					}

					if (currentGoal.getWrist().angle.getRadian() - prevState.getWrist().angle.getRadian() > 0) {
						nextElbow = prevState.getWrist().angle.getRadian() + wristRadPerSecond / deltaT;
					} else {
						nextElbow = prevState.getWrist().angle.getRadian() - wristRadPerSecond / deltaT;
					}

					toReturn.add(new SuperStructureState(new ElevatorState(LengthKt.getInch(nextHeight)),
							new RotatingArmState(RoundRotation2d.getRadian(nextElbow)),
							new RotatingArmState(RoundRotation2d.getRadian(nextWrist))));
				}
			}
		}

		//TODO this should add in-between states to the list  so they match up with the trajects
		//wps should be equal in length to bigPath
		//toReturn should be equal in length to path
		return toReturn;
	}

	public void writeToCSV(String file, ArrayList<Translation2d> path) {

		try {
			FileWriter fw = new FileWriter(file);
			PrintWriter pw = new PrintWriter(fw, true);

			pw.println("x,y");
			for (Translation2d t : path) {
				pw.println(t.getX().getFeet() + "," + t.getY().getFeet());
			}

			// pw.print("adsffdsaadsfdsfaadsffads1");

			pw.close();
		} catch (IOException ioe) {
			System.out.println(ioe);
		}

	}

	public void writeToAngleCSV(String file, ArrayList<SuperStructureState> path) {

		try {
			FileWriter fw = new FileWriter(file);
			PrintWriter pw = new PrintWriter(fw, true);

			pw.println("height,elbowAngle,wristAngle");
			for (SuperStructureState t : path) {
				pw.println(t.getElevatorHeight().getFeet() + "," + t.getElbowAngle().getRadian() + "," + t.getWrist().angle.getRadian());
			}

			// pw.print("adsffdsaadsfdsfaadsffads1");

			pw.close();
		} catch (IOException ioe) {
			System.out.println(ioe);
		}

	}

	public ArrayList<Translation2d> trajectToArrayList(TimedTrajectory<Pose2dWithCurvature> traject) {
		List<TimedEntry<Pose2dWithCurvature>> points = traject.getPoints();

		ArrayList<Translation2d> toReturn = new ArrayList<Translation2d>();

		for (TimedEntry<Pose2dWithCurvature> point : points) {
			toReturn.add(point.getState().getPose().getTranslation());
		}

		return toReturn;

	}

	public ArrayList<Translation2d> TwoHatchOneCargoLeft(char startPos, char side) {

		ArrayList<Translation2d> points = new ArrayList<Translation2d>();

		HeldPiece cPiece = HeldPiece.HATCH; // we do start with a hatch
		String cStart = "hab" + startPos;
		String mLoadingStation = "loading" + side;

		/* Get a trajectory to move to the cargo ship */
		TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "cargoM" + side); //current trajectory from hashmap in Trajectorie
		// Logger.log("first trajectory to cargo: " + traject.toString());
		points.addAll(trajectToArrayList(traject));

		/* Move from middle of cargo ship to loading station on the same side to pick up a hatch */
		cStart = "cargoM" + side;
		cPiece = HeldPiece.NONE;

		traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + mLoadingStation); //current trajectory from hashmap in Trajectorie
		points.addAll(trajectToArrayList(traject));

		/* Go right up to the cargo ship from the loading station */
		cStart = mLoadingStation;
		cPiece = HeldPiece.HATCH;

		traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "cargo" + side + '1'); //current trajectory from hashmap in Trajectorie
		points.addAll(trajectToArrayList(traject));

		// turn 90 degrees to face the goal
		// new TurnInPlace(Trajectories.locations.get("cargo" + side + '1').component2(), true)); // TODO check the angle math here! 

		/* Go from cargo side 1 to the depot */
		cStart = "cargo" + side + '1';
		cPiece = HeldPiece.NONE;

		traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "depot" + side); //current trajectory from hashmap in Trajectorie
		points.addAll(trajectToArrayList(traject));

		/* Go from depot to cargo ship ~~2~~ 1 darnit you're right. Thanks 10pm me */
		cStart = "depot" + side;
		cPiece = HeldPiece.CARGO;
		traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + "cargo" + side + '1'); //current trajectory from hashmap in Trajectorie
		points.addAll(trajectToArrayList(traject));

		return points;

	}

	public ArrayList<Translation2d> FullRocket(char startPos, char side) {

		ArrayList<Translation2d> points = new ArrayList<Translation2d>();

		String mLoadingStation = "loading" + side;
		String mCloseRocket = "rocket" + side + '1';
		String mFarRocket = "rocket" + side + '3';
		String mCargoPort = "rocket" + side + '2';

		HeldPiece cPiece = HeldPiece.HATCH; // we start with a hatch
		String cStart = "hab" + startPos;

		/* Get a trajectory to move to the rocket ship close side (THIS IS BACKWARDS/REVERSED!) */
		TimedTrajectory<Pose2dWithCurvature> traject = Trajectories.generatedHGTrajectories.get(cStart + " to " + mCloseRocket); //current trajectory from hashmap in Trajectorie
		if (traject == null)
			return null;
		points.addAll(trajectToArrayList(traject));

		cPiece = HeldPiece.NONE;

		/* Get a trajectory to move from close side cargo ship to loading station*/
		traject = Trajectories.generatedHGTrajectories.get(mCloseRocket + " to " + mLoadingStation); //current trajectory from hashmap in Trajectorie
		if (traject == null)
			return null;
		points.addAll(trajectToArrayList(traject));

		cPiece = HeldPiece.HATCH;

		/* Get a trajectory to move from loading station to far side rocket (this is 2 hatches) */
		traject = Trajectories.generatedHGTrajectories.get(mLoadingStation + " to " + mFarRocket); //current trajectory from hashmap in Trajectorie
		if (traject == null)
			Logger.log("Trajectory doesn't exist! ur bad.");
		points.addAll(trajectToArrayList(traject));

		cPiece = HeldPiece.NONE;

		/* Get a trajectory to move back from far rocket to loading station */
		traject = Trajectories.generatedHGTrajectories.get(mFarRocket + " to " + mLoadingStation); //current trajectory from hashmap in Trajectorie
		if (traject == null)
			Logger.log("Trajectory doesn't exist! ur bad.");
		points.addAll(trajectToArrayList(traject));

		/* Get a trajectory to move from loading station to far side rocket (this is 3 hatches) */
		traject = Trajectories.generatedHGTrajectories.get(mLoadingStation + " to " + mFarRocket); //current trajectory from hashmap in Trajectorie
		if (traject == null)
			Logger.log("Trajectory doesn't exist! ur bad.");
		points.addAll(trajectToArrayList(traject));

		cPiece = HeldPiece.NONE;

		/* Get a trajectory to move back from far rocket to loading station */
		traject = Trajectories.generatedHGTrajectories.get(mFarRocket + " to " + mLoadingStation); //current trajectory from hashmap in Trajectorie
		if (traject == null)
			Logger.log("Trajectory doesn't exist! ur bad.");
		points.addAll(trajectToArrayList(traject));

		cPiece = HeldPiece.HATCH;

		/* Get a trajectory to move from loading station to far side rocket (this is 4 hatches) */
		traject = Trajectories.generatedHGTrajectories.get(mLoadingStation + " to " + mFarRocket); //current trajectory from hashmap in Trajectorie
		if (traject == null)
			Logger.log("Trajectory doesn't exist! ur bad.");
		points.addAll(trajectToArrayList(traject));

		/* ------------------------------------------------------------------ This is all the far placed ------------------------------------------------------------------ */

		cPiece = HeldPiece.NONE;

		/* Get a trajectory to move back from far rocket to loading station */
		traject = Trajectories.generatedHGTrajectories.get(mFarRocket + " to " + mLoadingStation); //current trajectory from hashmap in Trajectorie
		if (traject == null)
			Logger.log("Trajectory doesn't exist! ur bad.");
		points.addAll(trajectToArrayList(traject));

		/* Get a trajectory to move to the rocket ship close side (THIS IS BACKWARDS/REVERSED!) */
		traject = Trajectories.generatedHGTrajectories.get(mLoadingStation + " to " + mCloseRocket); //current trajectory from hashmap in Trajectorie
		if (traject == null)
			return null;
		points.addAll(trajectToArrayList(traject));

		cPiece = HeldPiece.NONE;

		/* Get a trajectory to move from close side cargo ship to loading station*/
		traject = Trajectories.generatedHGTrajectories.get(mCloseRocket + " to " + mLoadingStation); //current trajectory from hashmap in Trajectorie
		if (traject == null)
			return null;
		points.addAll(trajectToArrayList(traject));

		/* Get a trajectory to move from loading station to the rocket ship close side for the last bloody time*/
		traject = Trajectories.generatedHGTrajectories.get(mLoadingStation + " to " + mCloseRocket); //current trajectory from hashmap in Trajectorie
		if (traject == null)
			return null;
		points.addAll(trajectToArrayList(traject));

		/* ------------------------------------------------------------------ The rocket hass all 6 hatches on now ------------------------------------------------------------------ */

		return points;

	}

}
