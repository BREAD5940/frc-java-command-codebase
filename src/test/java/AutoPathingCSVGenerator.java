import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.junit.jupiter.api.Test;

import frc.robot.commands.auto.AutoMotion.HeldPiece;
import frc.robot.commands.auto.Trajectories;
import frc.robot.lib.Logger;

public class AutoPathingCSVGenerator {

	@Test
	public void twoHatchLLtest() {
		Trajectories.generateAllTrajectories(false);
		ArrayList<Translation2d> path = TwoHatchOneCargoLeft('L', 'L');
		writeToCSV("twoHatchLLtest", path);

		// path = TwoHatchOneCargoLeft('R', 'R');
		// writeToCSV("twoHatchRRest", path);
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

		HeldPiece cPiece = HeldPiece.HATCH; // we start with a hatch
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
		// points.addAll(trajectToArrayList(traject));

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

		/* ------------------------------------------------------------------ The rocket is now full ------------------------------------------------------------------ */

		return points;

	}

}
