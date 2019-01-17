import static org.junit.Assert.*;

import org.junit.jupiter.api.Test;

import frc.robot.lib.motion.followers.PurePursuitFollower;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;

public class PurePursuitFollowerTest {
  @Test
  public void testCurvature() {
    System.out.println("Curature: " + curvatureFromPathfinder());
  }

  @Test
  public void testTrajectory() {

    

  }














  public double curvatureFromPathfinder() {

    /*
    To calculate curvature at a point we must first find the radius of 
      the circle that intersects this point, the point ahead of it, and the point 
      behind it. Once we have this, curvature is simply 1 / radius. From the pdf:
    Given  P(x1, y1), Q(x2, y2), and R(x3, y3):
    k  = 0.5 * ( x_1^2 + y_1^2 - Math.pow(x_2, 2) - y_^2)/(x_1 - x_2)
    k_2 = (y_1 - y_2)/(x_1 - x_2)
    b = 0.5 * (Math.pow(x_2, 2) - 2 * x_2 * k_1 + Math.pow(y_2, 2) - Math.pow(x_3, 2) + 2 * x_3 * k_1 - Math.pow(y_3, 2)) / (x_3 * k_2 - y_3 + y_2 - x_2 * k_2)
    a = k_1 - k_2 * b
    r = sqrt((x_1 - a)^2 + (y_1 - b)^2)
    curvature = 1/r

    However, consider that if x_1 = x_2 you divide by zero. (try adding 0.0001 to x?)
    Also, if 1/r = NaN, the curvature is zero.
    Furthermore, the first and last points don't have any preceding points, so just
      set their curvature to zero.
    */
    double k_1, k_2, a, b, r;

    // if ( (index == path.length()) || (index == 0) ) {
    //   return 0;
    // }

    // Point point1 = new Point(path.get(index-1).x,path.get(index-1).y);
    // Point point2 = new Point(path.get(index).x,path.get(index).y);
    // Point point3 = new Point(path.get(index+1).x,path.get(index+1).y);
    
    // double x_1 = point1.x;
    // double x_2 = point2.x;
    // double x_3 = point3.x;

    // double y_1 = point1.y;
    // double y_2 = point2.y;
    // double y_3 = point3.y;

    // double x_1 = points[0][0];
    // double x_2 = points[0][1];
    // double x_3 = points[0][2];

    // double y_1 = points[1][0];
    // double y_2 = points[1][1];
    // double y_3 = points[1][2];

    double x_1 = 0;//points[0][0];
    double x_2 = 1;//points[0][1];
    double x_3 = 2;//points[0][2];

    double y_1 = 0;//points[1][0];
    double y_2 = 1;//points[1][1];
    double y_3 = 2;//points[1][2];


    k_1  = 0.5 * ( Math.pow(x_1,2) + Math.pow(y_1,2) - Math.pow(x_2,2) - Math.pow(y_2,2))/(x_1 - x_2);
    k_2 = (y_1 - y_2)/(x_1 - x_2);
    b = 0.5 * (Math.pow(x_2, 2) - 2 * x_2 * k_1 + Math.pow(y_2, 2) - Math.pow(x_3, 2) + 2 * x_3 * k_1 - Math.pow(y_3, 2)) / (x_3 * k_2 - y_3 + y_2 - x_2 * k_2);
    a = k_1 - k_2 * b;
    r = Math.sqrt(Math.pow((x_1 - a),2) + Math.pow((y_1 - b),2));

    double curvature = 1.0/r;

    if (Double.isNaN(curvature)) {
      System.out.println("Curvature is nan");
      return 0;
    }

    return 1/r;
  }
    
  
}