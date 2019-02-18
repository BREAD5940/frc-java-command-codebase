package frc.robot.lib.obj;


public class RoundRotation2d {
	protected double value;
	protected double cosine;
	protected double sine;

	public RoundRotation2d(RoundRotation2d n){
		this.value = n.value;
		this.cosine = Math.cos(this.getRadian());
		this.sine = Math.sin(this.getRadian());
	}

	private RoundRotation2d(double deg){
		this.value = deg;
	}

	public static RoundRotation2d getDegree(double reciever){
		return new RoundRotation2d(reciever);
	}

	public static RoundRotation2d getRadian(double reciever){
		return new RoundRotation2d(reciever*(180/Math.PI));
	}

	public double getDegree(){
		return this.value;
	}

	public double getRadian(){
		return this.value*(Math.PI/180);
	}

	public double getCos(){
		return this.cosine;
	}

	public double getSin(){
		return this.sine;
	}
	
}
