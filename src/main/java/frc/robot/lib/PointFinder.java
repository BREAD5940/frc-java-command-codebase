package frc.robot.lib;

import java.util.ArrayList;

import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.opencv.core.Point;

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
        // System.out.println("found a topmost point! it's " + tempPoint.toString());
        topPoints.add(tempPoint);
        points.remove(tempPoint); // this should remove the current max from the list
        tempPoint = new Translation2d(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

        // Loop through all the y coordinates, find the topmost one
        for(int i=0; i<points.size(); i++) {
            if(points.get(i).getY().getValue() > tempPoint.getY().getValue() ) tempPoint = points.get(i) ;
        }
        // System.out.println("found a second topmost point! it's " + tempPoint.toString());
        topPoints.add(tempPoint);
        points.remove(tempPoint); // this should remove the current max from the list

        /* Now we should have only two left. Since we already added the two greatest points above, we know these are on the bottom. So just add them */
        bottomPoints = points; // TODO check me that it works as intended
        // System.out.println("BottomPoints contains: -------");
        // for(Translation2d t : bottomPoints) System.out.println(t.toString());
        // System.out.println("---------");

        /* Now we have them ordered by Y coordinate, we should check what order they should be in in terms of X axis */
        if(topPoints.get(0).getX().getValue() < topPoints.get(1).getX().getValue()) /* if the first index is more leftward than the second */ {
            // we set the top left coordinate to index zero and to right to index one
            topLeft = topPoints.get(0);
            topRight = topPoints.get(1);
            // System.out.println("Index zero is less than index one - topLeft is " + topLeft.toString() + " and topRight is " + topRight.toString());
        }
        else {
            // we have the inverse of above
            topLeft = topPoints.get(1);
            topRight = topPoints.get(0);
            // System.out.println("First check failed. So topLeft is " + topLeft.toString() + " and topRight is " + topRight.toString());
        }

        // We do the same thing for bottom points
        if(bottomPoints.get(0).getX().getValue() < bottomPoints.get(1).getX().getValue()) /* if the first index is more leftward than the second */ {
            // we set the top left coordinate to index zero and to right to index one
            bottomLeft = bottomPoints.get(0);
            bottomRight = bottomPoints.get(1);
            // System.out.println("Index zero is less than index one - bottomLeft is " + bottomLeft.toString() + " and bottomRight is " + bottomRight.toString());
        }
        else {
            // we have the inverse of above
            bottomLeft = bottomPoints.get(1);
            bottomRight = bottomPoints.get(0);
            // System.out.println("First check failed. So bottomLeft is " + bottomLeft.toString() + " and bottomRight is " + bottomRight.toString());
        }


}

public Point getTopLeft() {
    return new Point(topLeft.getX().getValue(), topLeft.getY().getValue());
}

public Point getTopRight() {
    return new Point(topRight.getX().getValue(), topRight.getY().getValue());
}

public Point getBottomLeft() {
    return new Point(bottomLeft.getX().getValue(), bottomLeft.getY().getValue());
}

public Point getBottomRight() {
    return new Point(bottomRight.getX().getValue(), bottomRight.getY().getValue());
}

}
