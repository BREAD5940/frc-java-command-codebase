package frc.robot.commands.subsystems.drivetrain;

import org.ghrobotics.lib.mathematics.units.LengthKt;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.lib.Logger;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.planners.SuperstructurePlanner;
import frc.robot.states.ElevatorState;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class ZeroSuperStructure extends Command {
	String p;
	SuperStructure struc;

	public ZeroSuperStructure(String piece) {
		this.p = piece;
		// requires(SuperStructure.getInstance());
		setInterruptible(false);
		struc = SuperStructure.getInstance();
		// Command comm = struc.getCurrentCommand();
		// if (comm != null)
			// comm.cancel();
	}

	@Override
	public void initialize() {
	
		SuperStructureState old = struc.lastState;

		System.out.println("Zeroing encoder to " + p);
		if (p.equals("elevator")) {
			Logger.log("zeroing elevator");
			SuperStructure.getInstance().getElevator().zeroEncoder();
			old.elevator = new ElevatorState();
		} else if (p.equals("elbow")) {
			Logger.log("zeroing elbow");
			SuperStructure.getInstance().getElbow().getMaster().setSensorPosition(RoundRotation2d.getDegree(0));
			old.getElbow().setAngle(new RoundRotation2d());
		} else if (p.equals("wrist")) {
			Logger.log("zeroing wrist");
			SuperStructure.getInstance().getWrist().getMaster().setSensorPosition(RoundRotation2d.getDegree(0));
			old.getWrist().setAngle(new RoundRotation2d());
		} else if (p.equals("maxElevator")) {
			SuperStructure.getInstance().getElevator().getMaster().setSensorPosition(SuperstructurePlanner.top);
			old.getElevator().setHeight(SuperstructurePlanner.top);
		} else if (p.equals("maxWrist")) {
			SuperStructure.getInstance().getWrist().getMaster().setSensorPosition(SuperstructurePlanner.overallMaxWrist);
			old.getWrist().setAngle(SuperstructurePlanner.overallMaxWrist);
		} else if (p.equals("minWrist")) {
			SuperStructure.getInstance().getWrist().getMaster().setSensorPosition(SuperstructurePlanner.overallMinWrist);
			old.getWrist().setAngle(SuperstructurePlanner.overallMinWrist);
		} else if (p.equals("maxElbow")) {
			SuperStructure.getInstance().getWrist().getMaster().setSensorPosition(SuperstructurePlanner.overallMaxElbow);
			old.getWrist().setAngle(SuperstructurePlanner.overallMaxElbow);
		} else if (p.equals("minElbow")) {
			SuperStructure.getInstance().getWrist().getMaster().setSensorPosition(SuperstructurePlanner.overallMinElbow);
			old.getWrist().setAngle(SuperstructurePlanner.overallMinElbow);
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
		struc.getDefaultCommand().cancel();
		System.out.println("Restarting the default command");
		struc.getDefaultCommand().start();
	}

	@Override
	public void interrupted() {

	}
}
