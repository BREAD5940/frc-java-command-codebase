package frc.robot.commands.subsystems.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.planners.SuperstructurePlanner;
import frc.robot.subsystems.superstructure.SuperStructure;

public class ZeroSuperStructure extends Command {
	String p;
	SuperStructure struc;

	public ZeroSuperStructure(String piece) {
		this.p = piece;
		// requires(SuperStructure.getInstance());
		setInterruptible(false);
		 struc = SuperStructure.getInstance();
		Command comm = struc.getCurrentCommand();
		if (comm != null)
			comm.cancel();
	}

	@Override
	public void initialize() {
		System.out.println("Zeroing encoders.");
		if (p.equals("elevator")) {
			SuperStructure.getInstance().getElevator().zeroEncoder();
		} else if (p.equals("elbow")) {
			SuperStructure.getInstance().getElbow().getMaster().setSensorPosition(RoundRotation2d.getDegree(0));
		} else if (p.equals("wrist")) {
			SuperStructure.getInstance().getWrist().getMaster().setSensorPosition(RoundRotation2d.getDegree(0));
		} else if (p.equals("maxElevator")) {
			SuperStructure.getInstance().getElevator().getMaster().setSensorPosition(SuperstructurePlanner.top);
		} else if (p.equals("maxWrist")) {
			SuperStructure.getInstance().getWrist().getMaster().setSensorPosition(SuperstructurePlanner.overallMaxWrist);
		} else if (p.equals("minWrist")) {
			SuperStructure.getInstance().getWrist().getMaster().setSensorPosition(SuperstructurePlanner.overallMinWrist);
		} else if (p.equals("maxElbow")) {
			SuperStructure.getInstance().getWrist().getMaster().setSensorPosition(SuperstructurePlanner.overallMaxElbow);
		} else if (p.equals("minElbow")) {
			SuperStructure.getInstance().getWrist().getMaster().setSensorPosition(SuperstructurePlanner.overallMinElbow);
		}
	}

	@Override
	public void execute() {

	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void end() {
		struc.getCurrentCommand().cancel();
		struc.getDefaultCommand().start();
	}

	@Override
	public void interrupted() {

	}
}
