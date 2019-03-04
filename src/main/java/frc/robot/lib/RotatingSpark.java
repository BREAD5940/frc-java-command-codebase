package frc.robot.lib;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import frc.robot.lib.obj.RoundRotation2d;

public class RotatingSpark extends Spark implements Loggable {

	private final int ticksPerRotation;
	private final Encoder mEncoder;

	public RotatingSpark(int motorPort, int encoderChannelA, int encoderChannelB, boolean reverse, double ticksPerRotation) {
		super(motorPort);
		this.ticksPerRotation = (int) ticksPerRotation;
		mEncoder = new Encoder(encoderChannelA, encoderChannelB, reverse, Encoder.EncodingType.k4X);

		mEncoder.setMaxPeriod(.1);
		mEncoder.setMinRate(10);
		mEncoder.setDistancePerPulse(1);
		mEncoder.setSamplesToAverage(7); // todo tune
	}

	public RoundRotation2d getRotation() {
		var temp = mEncoder.getDistance() / ticksPerRotation;
		return RoundRotation2d.fromRotations(temp);
	}

	@Override
	public String getCSVHeader() {
		return "Speed, position";
	}

	@Override
	public String toCSV() {
		return super.getSpeed() + "," + getRotation().getDegree();
	}

}
