package frc.robot.lib;

import frc.robot.Robot;
// import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.math.coordinateSystems;

import java.lang.ArrayIndexOutOfBoundsException;

public class Odometry {
    double startAngle, startLeft, startRight;
    double currentAngle, deltaLeft, deltaRight, deltaCenter, deltaTheta, chordLen;
    double oldAngle, oldLeft, oldRight;
    double deltaX, deltaY, globalX, globalY, globalTheta;
    double[] polarCoordinates = new double[2];
    double Rc, Rl, Rr; // left right and center radii variables
    double pi = Math.PI;
    double[] xyArray = new double[2];
    double thatOneWeirdTriangleAngleBoi; // caleld Ax on the jpg MattsOdometryStuff.jpg
    /** The current loop iteration of the odometer */
    int i = 0;
    // TODO so this might overflow and break everything, so catch the error lol
    int logLen = 200000;
    double[][] locationLog = new double[3][logLen]; // in the form (x, y, theta)
    boolean arrayFull;

    double oldX, oldY, oldTheta;
    double dCenter, deltaPhi, newTheta, newX, newY;

    public Odometry() {
        this.startAngle = this.oldAngle = Robot.gyro.getAngle();
        // this.currentLeft = this.startLeft = this.oldLeft = Robot.drivetrain.getLeftDistance();
        // this.currentRight = this.startRight = this.oldRight = Robot.drivetrain.getRightDistance();
        this.locationLog[0][i] = 0;
        this.locationLog[1][i] = 0;
        this.locationLog[2][i] = startAngle;
        this.globalX = this.globalY = 0;
        for(int x=0; x<logLen; x++) {
            for (int y=0; y<3; y++) {
                try {
                    locationLog[y][x] = 0;
                } catch (ArrayIndexOutOfBoundsException e) {
                    System.out.println("array out of bounds exception on cleaning out location log array, oops ¯\\_(ツ)_/¯");
                }
            }
        }
        System.out.println("Odometry constructed, location log flushed! Call init at the start of the match to init odometry, and call update periodically after that");
    }

    /**
     * Call me when the match starts to set stuff up. MAKE SURE THAT THE DRIVETRAIN IS ZEROED!!
     * If not, this will ZERO THE DRIVETRAIN
     * again, this WILL MESS WITH ENCODERS by ZEROING THEM if they aren't
     */
    public void init() {
        this.locationLog[2][0] = Robot.gyro.getAngle();
        i++; // incrament i by 1
    }

    /**
     * Update the odometry. Returns nothing
     */
    public void update() {
        if ( i > (logLen - 1)) { return; } // make sure that everything doesn't break
        i++;

        deltaLeft = Robot.drivetrain.getLeftDistance();
        deltaRight = Robot.drivetrain.getRightDistance();
        currentAngle = Robot.gyro.getAngle();   
        oldAngle = globalTheta; // grab the old "global theta" value
        // deltaCenter = (deltaLeft + deltaRight)/2;
        // thatOneWeirdTriangleAngleBoi = (-1 * currentAngle)/2 + 135 + (90-oldAngle)/2;
        
        // deltaX = deltaCenter * Math.toDegrees(Math.sin(thatOneWeirdTriangleAngleBoi));
        // deltaY = deltaCenter * Math.toDegrees(Math.cos(thatOneWeirdTriangleAngleBoi));
        
        // this.globalX += deltaX;
        // this.globalY += deltaY;
        // this.globalTheta = currentAngle;
        

        // So this is try 2 of odometry. The general idea this time around
        // is to create a polar coordinate position relative to oldAngle = 0,
        // then transform it to match the global frame (still in polar), then
        // incrament/decrement the global pos by that much
        // TODO tests for negtive movement
        deltaTheta = currentAngle - oldAngle;
        Rl = (180 * deltaLeft)/(pi * deltaTheta);
        Rr = (180 * deltaLeft)/(pi * deltaTheta);
        Rc = (Rl + Rr) / 2;

        chordLen = Math.toDegrees(Math.sin(deltaTheta)) * Rc * 2;
        polarCoordinates[0] = deltaTheta;
        polarCoordinates[1] = chordLen;

        // rotate the pose, because right now we are just using delta theta, 
        // not the global reference frame theta
        polarCoordinates[1] += oldAngle;



        oldAngle = currentAngle;
        updateLocLog(globalX, globalY, currentAngle); // call to update the log and also increase i by 1
    }




    /**
     * Update the location log, now with 100% more error catching!
     * @param globalXpos of the robot
     * @param globalYpos of the robot
     * @param globalTheta of the robot
     */
    public void updateLocLog(double globalXpos, double globalYpos, double globalTheta) {
        if(!arrayFull) {
            try {
                this.locationLog[0][i] = globalXpos;
                this.locationLog[1][i] = globalYpos;
                this.locationLog[2][i] = globalTheta;
            } catch (ArrayIndexOutOfBoundsException e) {
                System.out.println("Error on odometry location log update: array is full of values! I therefore cannot write to it. This is a bad thing^tm to happen");
                arrayFull = true;
            }
            i++;
        }
    }

}