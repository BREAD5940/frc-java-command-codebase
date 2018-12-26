  
package frc.robot.lib;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

import com.ctre.phoenix.motion.TrajectoryPoint;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.RobotConfig;
import frc.robot.lib.EncoderLib;

public class MotionProfileLib {
  // Misc methods for basic motion profiling
  // Credit to RobotThatFollows from team 1757

    /**
     * <p>
     * Reads and processes individual data points from a CSV file.
     *
     * @param path the path of the file with the data points
     * @return an ArrayList of double[] with each data point
     */
    public static ArrayList<double[]> readCSVMotionProfileFile(String path) {

      ArrayList<double[]> pathSegments = new ArrayList<>();

      try (BufferedReader br = new BufferedReader(new FileReader(path))) {

          String line;
          String csvDelimiter = ",";

          while ((line = br.readLine()) != null) {
              String[] segment = line.split(csvDelimiter);

              double[] convertedSegment = Arrays.stream(segment)
                      .mapToDouble(Double::parseDouble)
                      .toArray();

              pathSegments.add(convertedSegment);
          }

      } catch (IOException ex) {
          DriverStation.reportError("Unable to read motion profile file!", true);
      }

      return pathSegments;

  }   

    /**
     * Processes and feeds the generated data points one-by-one into the Talon SRX MPB.
     *
     * @param talonSRX the Talon SRX device reference
     * @param profile the generated trajectory extracted from a CSV file
     */
    private void loadTrajectoryToTalon(TalonSRX talonSRX, ArrayList<double[]> profile, double effective_diameter) {
      TrajectoryPoint point = new TrajectoryPoint();

      for (int i = 0; i < profile.size(); i++) {
          point.position = EncoderLib.distanceToRaw(profile.get(i)[0], effective_diameter, RobotConfig.POSITION_PULSES_PER_ROTATION);     // meters -> rotations -> ticks
          point.velocity = EncoderLib.distanceToRaw(((profile.get(i)[1]) / 10.0), effective_diameter, RobotConfig.POSITION_PULSES_PER_ROTATION);     // meters/second -> ticks/sec -> ticks/100ms
        //   point.timeDur = TrajectoryPoint.TrajectoryDuration.Trajectory_Duration_50ms;
        //   point.profileSlotSelect0 = 0;

          point.zeroPos = i == 0;
          point.isLastPoint = (i + 1) == profile.size();

          talonSRX.pushMotionProfileTrajectory(point);

      }

      System.out.println("Loaded Trajectory");

  }


}