package frc.robot.lib;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.lang.System.Logger;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Arrays;
import java.util.Date;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.RobotConfig;

/**
 * Logs stuff to a CSV file on the RoboRIO. Hopefully won't crash the code if the file can't
 * be written to. Depends on CSVUtils.java
 * 
 * @author Matthew Morley
 */
public class TerribleLogger {
	/**
	 * The System.nanoTime of the start of the robot.
	 */
    double startTime = getFPGAtime();

    boolean writeException = true;   
    public FileWriter writer;
    
	/**
	 * The running time of the robot in milliseconds starting from zero.
	 * @return The runtime.
	 */
	public double getRuntime() {
		return getFPGAtime() - startTime;
	}

    /**
     * Create an instance of the logger. Make sure that the filepath ends with a / lol
     */
    public TerribleLogger(){
        try {
            String date = getDate().toString();
            String filepath = RobotConfig.logging.default_filepath + date + ".csv";
            FileWriter writer = new FileWriter(filepath);
            String[] initialData = RobotConfig.logging.data_headers;
            CSVUtils.writeLine(writer, Arrays.asList(initialData));
            writeException = false;
        }
        catch (IOException ioe)
        {
            System.out.println("IOException when initilizing the log! See stacktrace");
            ioe.printStackTrace();
            writeException = true;
        }
        
    }

    public void update() {
        if(!writeException){
            try {
            String[] data = {
                String.valueOf(getFPGAtime()), 

                String.valueOf(Robot.drivetrain.getLeftDistance()), 
                String.valueOf(Robot.drivetrain.getLeftVelocity()), 
                String.valueOf(EncoderLib.rawToDistance(Robot.drivetrain.tRaw_l, 
                    RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION,
                    RobotConfig.driveTrain.left_wheel_effective_diameter)),
                String.valueOf(Robot.drivetrain.tVoltage_l),

                String.valueOf(Robot.drivetrain.getRightDistance()), 
                String.valueOf(Robot.drivetrain.getRightVelocity()), 
                String.valueOf(EncoderLib.rawToDistance(Robot.drivetrain.tRaw_r, 
                    RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION,
                    RobotConfig.driveTrain.right_wheel_effective_diameter)),
                String.valueOf(Robot.drivetrain.tVoltage_r),

                String.valueOf(Robot.gyro.getAngle()), 
                String.valueOf(Robot.gyro.getRate()),
                String.valueOf(RobotController.getBatteryVoltage()),
                String.valueOf(Robot.m_oi.getForwardAxis()),
                String.valueOf(Robot.m_oi.getTurnAxis()),
                String.valueOf(Robot.m_oi.getIntakeSpeed())
            };
            CSVUtils.writeLine(writer, Arrays.asList(data));
            writer.flush();
            } catch (IOException e) {
                System.out.println("IOException when updating the log! See stacktrace");
                e.printStackTrace();
                writeException = true;
            }
        } else {
            System.out.println("Cannot update log, write exception occured! Gunna just catch this error... :D");
        }
        
    }


    private Date getDate(){
        DateFormat dateFormat = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss");
        Date date = new Date();
        System.out.println(dateFormat.format(date)); //2016/11/16 12:08:43
        return date;
    }
    /**
     * Return the current time in seconds per the FPGA
     * @return run time in seconds
     */
    private double getFPGAtime() {
        return Timer.getFPGATimestamp();
    }
    
}