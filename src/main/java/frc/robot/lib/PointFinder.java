package frc.robot.lib;

import java.util.ArrayList;

import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d;
import org.opencv.core.Point;

public class PointFinder {

    // double[] xLoc, yLoc;
    Point topLeft, topRight, bottomLeft, bottomRight;
    boolean kYAxisFlipped = false; // make this true if you're using a limelight boi
    private static final int kReallyBig = 100000000;

    public PointFinder() {
        this(false);
    }

    public PointFinder(boolean flipped) {
        kYAxisFlipped = flipped;
    }

    public void calculate(double[] xLoc, double[] yLoc) {
        // first, find the two coordinates that are at the bottom and the top
        // point1, point2, point3, point4;
        ArrayList<Point> points = new ArrayList<Point>();

        points.add(new Point(xLoc[0], yLoc[0]));
        points.add(new Point(xLoc[1], yLoc[1]));
        points.add(new Point(xLoc[2], yLoc[2]));
        points.add(new Point(xLoc[3], yLoc[3]));

        ArrayList<Point> bottomPoints = new ArrayList<Point>();
        ArrayList<Point> topPoints = new ArrayList<Point>();
        Point tempPoint = new Point(-kReallyBig, -kReallyBig);

        // Loop through all the y coordinates, find the topmost one
        for(int i=0; i<points.size(); i++) {
            if(points.get(i).y > tempPoint.y ) tempPoint = points.get(i) ;
        }
        // System.out.println("found a topmost point! it's " + tempPoint.toString());
        topPoints.add(tempPoint);
        points.remove(tempPoint); // this should remove the current max from the list
        tempPoint = new Point(-kReallyBig, -kReallyBig);

        // Loop through all the y coordinates, find the topmost one
        for(int i=0; i<points.size(); i++) {
            if(points.get(i).y > tempPoint.y ) tempPoint = points.get(i) ;
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
        if(topPoints.get(0).x < topPoints.get(1).x) /* if the first index is more leftward than the second */ {
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
        if(bottomPoints.get(0).x < bottomPoints.get(1).x) /* if the first index is more leftward than the second */ {
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
        return new Point(topLeft.x, topLeft.y);
    }

    public Point getTopRight() {
        return new Point(topRight.x, topRight.y);
    }

    public Point getBottomLeft() {
        return new Point(bottomLeft.x, bottomLeft.y);
    }

    public Point getBottomRight() {
        return new Point(bottomRight.x, bottomRight.y);
    }

}
