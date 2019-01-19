package frc.robot.lib;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

import java.lang.String;

import com.ctre.phoenix.motion.MotionProfileStatus;
import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.RobotConfig;
import frc.robot.lib.EncoderLib;

/**
 * This is some basic functions for motion profiling. Similar to pathfinder, but also
 * not nearly as good^tm. Contains methods for reading, preparing and sending motion
 * profiles to a Talon.
 * 
 * @author Matthew Morley
 */
public class MotionProfileLib {
  // Misc methods for basic motion profiling
  // Credit to RobotThatFollows from team 1757

  /**
   * Initiate a new motion path. This should be called on auto init
   * @param talon to configure
   * @param path to the csv file
   * @param effective_diameter of the drivetrain wheel
   * @param motionProfileStatus object
   */
  public static void motionProfileInit(TalonSRX talon, String path, double effective_diameter, MotionProfileStatus motionProfileStatus) {

    System.out.println("--- Loading trajectory to talon... ---");

    // MotionProfileStatus talonMPstatus = new MotionProfileStatus();
    talon.getMotionProfileStatus(motionProfileStatus);

    ArrayList<double[]> profile = readCSVMotionProfileFile(path);

    System.out.println("Profile parsed from CSV! Size: " + profile.size());

    if (motionProfileStatus.hasUnderrun)
      ((IMotorController) motionProfileStatus).clearMotionProfileHasUnderrun(0);

    talon.clearMotionProfileTrajectories();
    talon.changeMotionControlFramePeriod(0);

    loadTrajectoryToTalon(talon, profile, effective_diameter);

    System.out.println("--- Trajectory loaded to talon! ---");
  }



    /**
     * <p>
     * Reads and processes individual data points from a CSV file.
     *
     * @param path the path of the file with the data points
     * @return an ArrayList of double[] with each data point
     */
  public static ArrayList<double[]> readCSVMotionProfileFile(String path) {

    ArrayList<double[]> pathSegments = new ArrayList<>();
    boolean isfirst = true;

    try (BufferedReader br = new BufferedReader(new FileReader(path))) {

      String line;
      String csvDelimiter = ",";

      while ((line = br.readLine()) != null) {
        String[] segment = line.split(csvDelimiter);
        if (!(isfirst)) {
          double[] convertedSegment = Arrays.stream(segment)
            .mapToDouble(Double::parseDouble)
            .toArray();
          pathSegments.add(convertedSegment);
        }
        isfirst = false;
      }
      } catch (IOException ex) {
          System.out.println("Unable to read motion profile file!");
      }

      // Print out the csv for debugging
      System.out.println("Converted segments:");
      for(int x=0; x<pathSegments.size(); x++) {
        // double number = pathSegments.get(i)[1];
        System.out.println("Line number: " + x + " Position: " + pathSegments.get(x)[3]
          + " Velocity: " + pathSegments.get(x)[4] + " Angle: " + pathSegments.get(x)[7]);
      }
      return pathSegments;
    }

    /**
     * Processes and feeds the generated data points one-by-one into the Talon SRX MPB. Be sure
     * that the units in effective wheel diameter and the CSV file are the same!! Otherwise bad
     * things will happen, and you will be sad there is now another hole in the DRG
     *
     * @param talonSRX the Talon SRX device reference
     * @param profile the generated trajectory extracted from a CSV file
     */
    public static void loadTrajectoryToTalon(TalonSRX talonSRX, ArrayList<double[]> profile, double effective_diameter) {
      TrajectoryPoint point = new TrajectoryPoint();

      for (int i = 0; i < profile.size(); i++) {
        point.position = EncoderLib.distanceToRaw(profile.get(i)[4], effective_diameter, 
          RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION);     // meters -> rotations -> ticks
        point.velocity = EncoderLib.distanceToRaw(((profile.get(i)[5]) / 10.0), effective_diameter, 
          RobotConfig.driveTrain.POSITION_PULSES_PER_ROTATION);     // meters/second -> ticks/sec -> ticks/100ms
        point.headingDeg = profile.get(i)[7];
        point.timeDur = 50;
        point.profileSlotSelect0 = 0;

        point.zeroPos = i == 0; // This is true if i is zero??
        point.isLastPoint = (i + 1) == profile.size(); // This is true if i+1 is the size?
        talonSRX.pushMotionProfileTrajectory(point);
      }

      System.out.println("Loaded Trajectory");

  }


}
