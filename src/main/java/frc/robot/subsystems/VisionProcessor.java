package frc.robot.subsystems;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;

import frc.robot.lib.PointFinder;

public class VisionProcessor {

	private MatOfPoint3f mObjectPoints;
	private Mat mCameraMatrix;
	private MatOfDouble mDistortionCoefficients;
	PointFinder pointFinder;

	//   private NetworkTable mLimelightTable;

	public VisionProcessor(boolean isYAxisFlipped) {
		System.loadLibrary(org.opencv.core.Core.NATIVE_LIBRARY_NAME);

		// Define bottom right corner of left vision target as origin
		mObjectPoints = new MatOfPoint3f(
				new Point3(0.0, 0.0, 0.0), // bottom right
				new Point3(-1.9363, 0.5008, 0.0), // bottom left
				new Point3(-0.5593, 5.8258, 0.0), // top-left
				new Point3(1.377, 5.325, 0.0) // top-right
		);

		// Thank you to xForceDee on CD! https://www.chiefdelphi.com/t/limelight-real-world-camera-positioning/343941/18
		// units are in inches (gah)!
		mCameraMatrix = Mat.eye(3, 3, CvType.CV_64F);
		mCameraMatrix.put(0, 0, 2.5751292067328632e+02);
		mCameraMatrix.put(0, 2, 1.5971077914723165e+02);
		mCameraMatrix.put(1, 1, 2.5635071715912881e+02);
		mCameraMatrix.put(1, 2, 1.1971433393615548e+02);

		mDistortionCoefficients = new MatOfDouble(2.9684613693070039e-01, -1.4380252254747885e+00, -2.2098421479494509e-03, -3.3894563533907176e-03, 2.5344430354806740e+00);

		pointFinder = new PointFinder(isYAxisFlipped);
	}

	public void update(double[] cornX, double[] cornY) {
		// double[] cornX = new double[]{0*d, -1.9363*d, -0.5583*d, 1.377*d}; //mLimelightTable.getEntry("tcornx").getDoubleArray(new double[0]);
		// double[] cornY = new double[]{0*d, 0.501*d, 5.8258*d, 5.325*d}; //mLimelightTable.getEntry("tcorny").getDoubleArray(new double[0]);

		String logData = "Contour corners: ";
		for (int i = 0; i < cornX.length; i++) {
			logData = logData + " (" + cornX[i] + ", " + cornY[i] + ") ----";
		}
		// System.out.println(logData);

		if (cornX.length != 4 || cornY.length != 4) {
			System.out.println("[ERROR] Could not find 4 points from image");
			return;
		}

		pointFinder.calculate(cornX, cornY);

		// print(String.format("Top left (%s) top right (%s) bottom left (%s) bottom right (%s)", 
		// pointFinder.getTopLeft(), 
		//     pointFinder.getTopRight(), pointFinder.getBottomLeft(), 
		//     pointFinder.getBottomRight()));

		// SmartDashboard.putString("Corners of the contour", String.format("Top left (%s) top right (%s) bottom left (%s) bottom right (%s)", 
		//       pointFinder.getTopLeft(), pointFinder.getTopRight(), pointFinder.getBottomLeft(), pointFinder.getBottomRight()));

		MatOfPoint2f imagePoints = new MatOfPoint2f(
				pointFinder.getBottomRight(),
				pointFinder.getBottomLeft(),
				pointFinder.getTopLeft(),
				pointFinder.getTopRight());

		// Mat rotationVector = new Mat();
		// Mat translationVector = new Mat();
		// Calib3d.solvePnP(mObjectPoints, imagePoints, mCameraMatrix, mDistortionCoefficients, rotationVector, translationVector);

		// System.out.println("rotationVector: " + rotationVector.dump());
		// System.out.println("translationVector: " + translationVector.dump());

		Mat rotationVector = new Mat();
		Mat translationVector = new Mat();
		Calib3d.solvePnP(mObjectPoints, imagePoints, mCameraMatrix, mDistortionCoefficients,
				rotationVector, translationVector);

		Mat rotationMatrix = new Mat();
		Calib3d.Rodrigues(rotationVector, rotationMatrix);

		// print("RotatrotationMatrixion mat:" + rotationMatrix.toString());
		// print("translationVector mat:" + translationVector.toString());
		// SmartDashboard.putString("OpenCV Rotation matrix", rotationMatrix.toString());

		Mat projectionMatrix = new Mat(3, 4, CvType.CV_64F);
		projectionMatrix.put(0, 0,
				rotationMatrix.get(0, 0)[0], rotationMatrix.get(0, 1)[0], rotationMatrix.get(0, 2)[0], translationVector.get(0, 0)[0],
				rotationMatrix.get(1, 0)[0], rotationMatrix.get(1, 1)[0], rotationMatrix.get(1, 2)[0], translationVector.get(1, 0)[0],
				rotationMatrix.get(2, 0)[0], rotationMatrix.get(2, 1)[0], rotationMatrix.get(2, 2)[0], translationVector.get(2, 0)[0]);

		Mat cameraMatrix = new Mat();
		Mat rotMatrix = new Mat();
		Mat transVect = new Mat();
		Mat rotMatrixX = new Mat();
		Mat rotMatrixY = new Mat();
		Mat rotMatrixZ = new Mat();
		Mat eulerAngles = new Mat();
		Calib3d.decomposeProjectionMatrix(projectionMatrix, cameraMatrix, rotMatrix,
				transVect, rotMatrixX, rotMatrixY, rotMatrixZ, eulerAngles);

		double rollInDegrees = eulerAngles.get(2, 0)[0];
		double pitchInDegrees = eulerAngles.get(0, 0)[0];
		double yawInDegrees = eulerAngles.get(1, 0)[0];

		// print("translationVector cols: " + transVect.cols());
		// print("translationVector rows: " + transVect.rows());
		print("translationVector: " + translationVector.get(0, 0)[0] + ", " + translationVector.get(1, 0)[0] + ", " + translationVector.get(2, 0)[0]);

		// System.out.println("rollInDegrees" + rollInDegrees);
		// System.out.println("pitchInDegrees" + pitchInDegrees);
		// System.out.println("yawInDegrees" + yawInDegrees);

	}

	public void print(String message) {
		System.out.println(message);
	}
}
