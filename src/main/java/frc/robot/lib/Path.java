package frc.robot.lib;

import frc.robot.states.SuperStructureState;

/**
 * This is a Path object for the motion of the superstructure. it uses
 * superstructurestates as waypoints
 */
public class Path {

	public SuperStructureState[] states;
	public double maxEleAccel, maxEleVel;
	public double maxWAccel, maxWVel;
	public double maxEAccel, maxEVel;

	public Path(SuperStructureState[] states, double mEleA, double mEleV, double mEA, double mEV, double mWA,
			double mWV) {
		this.states = states;
	}

	public Path(SuperStructureState[] states) {
		this(states, 1, 1, 1, 1, 1, 1); //defaults maxes to 1
	}

	public static double[][] inject(double[][] orig, int numToInject) {
		double morePoints[][];

		//create extended 2 Dimensional array to hold additional points
		morePoints = new double[orig.length + ((numToInject) * (orig.length - 1))][2];

		int index = 0;

		//loop through original array
		for (int i = 0; i < orig.length - 1; i++) {
			//copy first
			morePoints[index][0] = orig[i][0];
			morePoints[index][1] = orig[i][1];
			index++;

			for (int j = 1; j < numToInject + 1; j++) {
				//calculate intermediate x points between j and j+1 original points
				morePoints[index][0] = j * ((orig[i + 1][0] - orig[i][0]) / (numToInject + 1)) + orig[i][0];

				//calculate intermediate y points  between j and j+1 original points
				morePoints[index][1] = j * ((orig[i + 1][1] - orig[i][1]) / (numToInject + 1)) + orig[i][1];

				index++;
			}
		}

		//copy last
		morePoints[index][0] = orig[orig.length - 1][0];
		morePoints[index][1] = orig[orig.length - 1][1];
		index++;

		return morePoints;
	}

	public static double[][] smoother(double[][] path, double weight_data, double weight_smooth, double tolerance) {

		//copy array
		double[][] newPath = doubleArrayCopy(path);

		double change = tolerance;
		while (change >= tolerance) {
			change = 0.0;
			for (int i = 1; i < path.length - 1; i++)
				for (int j = 0; j < path[i].length; j++) {
					double aux = newPath[i][j];
					newPath[i][j] += weight_data * (path[i][j] - newPath[i][j]) + weight_smooth * (newPath[i - 1][j] + newPath[i + 1][j] - (2.0 * newPath[i][j]));
					change += Math.abs(aux - newPath[i][j]);
				}
		}

		return newPath;

	}

	public static double[][] doubleArrayCopy(double[][] arr) {

		//size first dimension of array
		double[][] temp = new double[arr.length][arr[0].length];

		for (int i = 0; i < arr.length; i++) {
			//Resize second dimension of array
			temp[i] = new double[arr[i].length];

			//Copy Contents
			for (int j = 0; j < arr[i].length; j++)
				temp[i][j] = arr[i][j];
		}

		return temp;

	}

	//id functions

	public SuperStructureState[] getStates() {
		return this.states;
	}

	public double getMaxAccel() {
		return this.maxEleAccel;
	}

	public double getMaxVelo() {
		return this.maxEleVel;
	}

}
