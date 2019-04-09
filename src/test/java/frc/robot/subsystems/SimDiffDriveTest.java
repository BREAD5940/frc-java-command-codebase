package frc.robot.subsystems;

import java.awt.Color;
import java.awt.Font;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.ghrobotics.lib.mathematics.twodim.control.RamseteTracker;
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.AccelerationKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.simulation.SimDifferentialDrive;
import org.ghrobotics.lib.simulation.SimFalconMotor;
import org.junit.Test;
import org.knowm.xchart.SwingWrapper;
import org.knowm.xchart.XYChart;
import org.knowm.xchart.XYChartBuilder;

import frc.robot.Constants;
import frc.robot.commands.auto.Trajectories;

public class SimDiffDriveTest {

	SimDifferentialDrive drive;

	@Test
	public void testTrajectoryTracker() {

		var leftMotor = new SimFalconMotor<Length>(LengthKt.getMeter(0));
		var rightMotor = new SimFalconMotor<Length>(LengthKt.getMeter(0));

		TrajectoryTracker trajectoryTracker = new RamseteTracker(Constants.kDriveBeta, Constants.kDriveZeta);

		drive = new SimDifferentialDrive(Constants.kLowGearDifferentialDrive, leftMotor, rightMotor, trajectoryTracker, 1.05);

		var rocketCToLoading = Arrays.asList(
				new Pose2d(LengthKt.getFeet(5.156),
						LengthKt.getFeet(17.712),
						Rotation2dKt.getDegree(0)),
				new Pose2d(LengthKt.getFeet(23.272),
						LengthKt.getFeet(18.321),
						Rotation2dKt.getDegree(-144.161)),
				new Pose2d(LengthKt.getFeet(15.83),
						LengthKt.getFeet(14.225),
						Rotation2dKt.getDegree(-120.339)),
				new Pose2d(LengthKt.getFeet(16.811),
						LengthKt.getFeet(2.199),
						Rotation2dKt.getDegree(-28.131))

		);

		var t_floorToRocketC = Trajectories.generateTrajectory(rocketCToLoading, Trajectories.kLowGearConstraints,
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(0.0)),
				VelocityKt.getVelocity(LengthKt.getFeet(6.5)),
				AccelerationKt.getAcceleration(LengthKt.getFeet(8.0)),
				false,
				true);

		trajectoryTracker.reset(t_floorToRocketC);

		// drive.robotPosition = purePursuitTracker.referencePoint!!.state.state.pose
		drive.setRobotPosition(trajectoryTracker.getReferencePoint().getState().getState().getPose());

		var currentTime = TimeUnitsKt.getSecond(0); //0.second
		var deltaTime = TimeUnitsKt.getMillisecond(20); //20.millisecond

		// ArrayList<Pose2d> poses = new ArrayList<>();

		List<Double> refXList = new ArrayList<>();
		List<Double> refYList = new ArrayList<>();

		List<Double> xList = new ArrayList<>();
		List<Double> yList = new ArrayList<>();

		while (!trajectoryTracker.isFinished()) {
			currentTime = currentTime.plus(deltaTime);
			drive.setOutput(trajectoryTracker.nextState(drive.getRobotPosition(), currentTime));
			drive.update(deltaTime);

			var drivePose = drive.getRobotPosition();

			var data = drivePose.getTranslation().getX().getFeet() + "," +
					drivePose.getTranslation().getY().getFeet() + "," +
					drivePose.getRotation().getDegree();

			xList.add(drivePose.getTranslation().getX().getFeet());
			yList.add(drivePose.getTranslation().getY().getFeet());

			refXList.add(trajectoryTracker.getReferencePoint().getState().getState().getPose().getTranslation().getX().getFeet());
			refYList.add(trajectoryTracker.getReferencePoint().getState().getState().getPose().getTranslation().getY().getFeet());

			System.out.println(data);

		}

		var totalTime = new DecimalFormat("#.###").format(t_floorToRocketC.getLastInterpolant().getSecond());

		var chart = new XYChartBuilder().width(1800).height(1520).title(totalTime + " seconds.")
				.xAxisTitle("X").yAxisTitle("Y").build();

		chart.getStyler().setMarkerSize(8);// = 8;
		chart.getStyler().setSeriesColors(new Color[]{Color.ORANGE, new Color(151, 60, 67)});// = arrayOf(Color.ORANGE, Color(151, 60, 67));

		chart.getStyler().setChartTitleFont(new Font("Kanit", 1, 40));
		chart.getStyler().setChartTitlePadding(15);

		chart.getStyler().setXAxisMin(1.0);
		chart.getStyler().setXAxisMax(26.0);
		chart.getStyler().setYAxisMin(1.0);
		chart.getStyler().setYAxisMax(26.0);

		chart.getStyler().setChartFontColor(Color.WHITE);
		chart.getStyler().setAxisTickLabelsColor(Color.WHITE);

		chart.getStyler().setLegendBackgroundColor(Color.GRAY);

		chart.getStyler().setPlotGridLinesVisible(true);
		chart.getStyler().setLegendVisible(true);

		chart.getStyler().setPlotGridLinesColor(Color.GRAY);
		chart.getStyler().setChartBackgroundColor(Color.DARK_GRAY);
		chart.getStyler().setPlotBackgroundColor(Color.DARK_GRAY);

		chart.addSeries("Trajectory", refXList, refYList);
		chart.addSeries("Robot", xList, yList);

		// SwingWrapper(chart).displayChart()
		new SwingWrapper<XYChart>(chart).displayChart();

		try {
			Thread.sleep(100000);
		} catch (Exception e) {
			e.printStackTrace();
		}

	}

}
