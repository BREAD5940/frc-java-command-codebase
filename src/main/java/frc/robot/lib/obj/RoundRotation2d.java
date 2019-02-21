package frc.robot.lib.obj;

import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;

public class RoundRotation2d {
	protected double value;
	protected double cosine;
	protected double sine;

	public RoundRotation2d(RoundRotation2d n) {
		this.value = n.value;
		this.cosine = Math.cos(this.getRadian());
		this.sine = Math.sin(this.getRadian());
	}

	private RoundRotation2d(double deg) {
		this.value = deg;
	}

	public RoundRotation2d() {
		this(0);
	}

	public static RoundRotation2d fromRotation2d(Rotation2d meme_) {
		return new RoundRotation2d(meme_.getDegree());
	}

	public static RoundRotation2d getDegree(double reciever) {
		return new RoundRotation2d(reciever);
	}

	public static RoundRotation2d getRadian(double reciever) {
		return new RoundRotation2d(reciever * (180 / Math.PI));
	}

	public RoundRotation2d times(double factor) {
		return new RoundRotation2d(this.value * factor);
	}

	public Rotation2d toRotation2d() {
		return Rotation2dKt.getDegree(this.value);
	}

	public double getDegree() {
		return this.value;
	}

	public double getRadian() {
		return this.value * (Math.PI / 180);
	}

	public double getCos() {
		return this.cosine;
	}

	public double getSin() {
		return this.sine;
	}

	public static RoundRotation2d fromRotations(double rotations) {
		return new RoundRotation2d(rotations * 360d);
	}

	public RoundRotation2d plus(RoundRotation2d other) {
		return new RoundRotation2d(this.value + other.getDegree());
	}

	public RoundRotation2d minus(RoundRotation2d other) {
		return new RoundRotation2d((this.value - other.value));
	}

	public double getRotations() {
		return getDegree() / 360d;
	}

}
