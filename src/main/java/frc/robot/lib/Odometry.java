package frc.robot.lib;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class Odometry {
    double startAngle, startLeft, startRight;
    double currentAngle, currentLeft, currentRight;
    double oldAngle, oldLeft, oldRight;
    private ADXRS450_Gyro gyro;
    double[] location; // in the form (x, y, theta)

    double oldX, oldY, oldTheta;
    double dCenter, deltaPhi, newTheta, newX, newY;

    public Odometry(ADXRS450_Gyro gyro) {
        this.gyro = gyro;
        this.startAngle = this.oldAngle = gyro.getAngle();
        this.currentLeft = this.startLeft = this.oldLeft = Robot.drivetrain.getLeftDistance();
        this.currentRight = this.startRight = this.oldRight = Robot.drivetrain.getRightDistance();
        this.location[0] = 0;
        this.location[1] = 0;
        this.location[2] = 0;
    }

    /**
     * 
     * @param leftDisplacement
     * @param rightDisplacement
     * @return displacement in the form [x, y, theta] relative to odometry init
     */
    public void update() {
        currentLeft = Robot.drivetrain.getLeftDistance();
        currentRight = Robot.drivetrain.getRightDistance();
        currentAngle = gyro.getAngle();
    }

}