package frc.robot.subsystems;

import java.awt.Point;
import java.awt.geom.Point2D;
import java.util.ArrayList;

import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point3;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionProcessor {

  private MatOfPoint3f mObjectPoints;
  private Mat mCameraMatrix;
  private MatOfDouble mDistortionCoefficients;

  private NetworkTable mLimelightTable;

  public VisionProcessor() {
      // Define bottom right corner of left vision target as origin
      mObjectPoints = new MatOfPoint3f(
              new Point3(0.0, 0.0, 0.0), // bottom right
              new Point3(-1.9363, 0.5008, 0.0), // bottom left
              new Point3(-0.5593, 5.8258, 0.0), // top-left
              new Point3(1.377, 5.325, 0.0) // top-right
      );

      mCameraMatrix = Mat.eye(3, 3, CvType.CV_64F);
      mCameraMatrix.put(0, 0, 2.5751292067328632e+02);
      mCameraMatrix.put(0, 2, 1.5971077914723165e+02);
      mCameraMatrix.put(1, 1, 2.5635071715912881e+02);
      mCameraMatrix.put(1, 2, 1.1971433393615548e+02);

      mDistortionCoefficients = new MatOfDouble(2.9684613693070039e-01, -1.4380252254747885e+00, -2.2098421479494509e-03, -3.3894563533907176e-03, 2.5344430354806740e+00);

      mLimelightTable = NetworkTableInstance.getDefault().getTable("limelight");
      mLimelightTable.getEntry("pipeline").setNumber(0); // use pipeline 0
      mLimelightTable.getEntry("camMode").setNumber(0); // vision mode
      mLimelightTable.getEntry("ledMode").setNumber(3); // force LED on
  }

  public void update() {
      double[] cornX = mLimelightTable.getEntry("tcornx").getDoubleArray(new double[0]);
      double[] cornY = mLimelightTable.getEntry("tcorny").getDoubleArray(new double[0]);

      String logData = "Contour corners: ";
      for(int i=0; i<cornX.length; i++) {
          logData = logData + " (" + cornX[i] + ", " + cornY[i] + ") ----";
      }
      System.out.println(logData);

      if (cornX.length != 4 || cornY.length != 4) {
          System.out.println("[ERROR] Could not find 4 points from image");
          return;
      }

      PointFinder pointFinder = new PointFinder(cornX, cornY);
      pointFinder.calculate();

      MatOfPoint2f imagePoints = new MatOfPoint2f(
              pointFinder.getBottomRight(), 
              pointFinder.getBottomLeft(),
              pointFinder.getTopLeft(), 
              pointFinder.getTopRight() //FIXME where are the definitions of these functions? _are_ there definitions for these functions?
       );

      Mat rotationVector = new Mat();
      Mat translationVector = new Mat();
      Calib3d.solvePnP(mObjectPoints, imagePoints, mCameraMatrix, mDistortionCoefficients, rotationVector, translationVector);

      System.out.println("rotationVector: " + rotationVector.dump());
      System.out.println("translationVector: " + translationVector.dump());
  }

  public class PointFinder {

    double[] xLoc, yLoc;
    Translation2d topLeft, topRight, bottomLeft, bottomRight;

    public PointFinder(double[] xCoords, double[] yCoords) {
        xLoc = xCoords;
        yLoc = yCoords;
    }

    public void calculate() {
        // first, find the two coordinates that are at the bottom and the top
        // point1, point2, point3, point4;
        ArrayList<Translation2d> points = new ArrayList<Translation2d>();

        points.add(new Translation2d(xLoc[0], yLoc[0]));
        points.add(new Translation2d(xLoc[1], yLoc[1]));
        points.add(new Translation2d(xLoc[2], yLoc[2]));
        points.add(new Translation2d(xLoc[3], yLoc[3]));



        ArrayList<Translation2d> bottomPoints = new ArrayList<Translation2d>();
        ArrayList<Translation2d> topPoints = new ArrayList<Translation2d>();
        Translation2d tempPoint = new Translation2d(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

        // Loop through all the y coordinates, find the topmost one
        for(int i=0; i<points.size(); i++) {
            if(points.get(i).getY().getValue() > tempPoint.getY().getValue() ) tempPoint = points.get(i) ;
        }
        topPoints.add(tempPoint);
        points.remove(tempPoint); // this should remove the current max from the list
        tempPoint = new Translation2d(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

        // Loop through all the y coordinates, find the topmost one
        for(int i=0; i<points.size(); i++) {
            if(points.get(i).getY().getValue() > tempPoint.getY().getValue() ) tempPoint = points.get(i) ;
        }
        topPoints.add(tempPoint);
        points.remove(tempPoint); // this should remove the current max from the list

        /* Now we should have only two left. Since we already added the two greatest points above, we know these are on the bottom. So just add them */
        bottomPoints = points; // TODO check me that it works as intended


        /* Now we have them ordered by Y coordinate, we should check what order they should be in in terms of X axis */
        if(topPoints.get(0).getX().getValue() < topPoints.get(1).getX().getValue()) /* if the first index is more leftward than the second */ {
            // we set the top left coordinate to index zero and to right to index one
            topLeft = topPoints.get(0);
            topRight = topPoints.get(1);
        }
        else {
            // we have the inverse of above
            topLeft = topPoints.get(1);
            topRight = topPoints.get(0);
        }

        // We do the same thing for bottom points
        if(bottomPoints.get(0).getX().getValue() < bottomPoints.get(1).getX().getValue()) /* if the first index is more leftward than the second */ {
            // we set the top left coordinate to index zero and to right to index one
            bottomLeft = topPoints.get(0);
            bottomRight = topPoints.get(1);
        }
        else {
            // we have the inverse of above
            bottomLeft = bottomPoints.get(1);
            bottomRight = bottomPoints.get(0);
        }


    }

    public double[] getTopLeft() {
        return new double[]{topLeft.getX().getValue(), topLeft.getY().getValue(), };
    }

    public double[] getTopRight() {
        return new double[]{topRight.getX().getValue(), topRight.getY().getValue(), };
    }
 
    public double[] getBottomLeft() {
        return new double[]{bottomLeft.getX().getValue(), bottomLeft.getY().getValue(), };
    }

    public double[] getBottomRight() {
        return new double[]{bottomRight.getX().getValue(), bottomRight.getY().getValue(), };
    }

  }

}