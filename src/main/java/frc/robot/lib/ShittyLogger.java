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

public class ShittyLogger {
	/**
	 * The System.nanoTime of the start of the robot.
	 */
    static long startTime = System.nanoTime();
    	
	/**
	 * The running time of the robot in milliseconds starting from zero.
	 * @return The runtime.
	 */
	public static long getRuntime() {
		
		return (System.nanoTime() - startTime)/1000;
		
	}

    /**
     * Create an instance of the logger. Make sure that the filepath ends with a / lol
     * @param filepath
     */
    public ShittyLogger(String filepath){
        init(filepath);
    }
    /**
     * Create an instance of the logger. Make sure that the filepath ends with a / lol
     */
    public ShittyLogger(){
        init(RobotConfig.logging.default_filepath);
    }

    public void init(String filepath){
        try {
            String date = getDate().toString();
            filepath = filepath + date + ".csv";
            FileWriter writer = new FileWriter(filepath);
            String[] initialData = RobotConfig.logging.data_headers;
            CSVUtils.writeLine(writer, Arrays.asList(initialData));
        }
        catch (IOException ioe)
        {
            System.out.println("Cannot open log file - maybe a log already exists?");        
        }
    }

    public void update() {
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
    }


    private Date getDate(){
        DateFormat dateFormat = new SimpleDateFormat("yyyy/MM/dd HH:mm:ss");
        Date date = new Date();
        System.out.println(dateFormat.format(date)); //2016/11/16 12:08:43
        return date;
    }
    
    private double getFPGAtime() {
        return Timer.getFPGATimestamp();
    }
    
}