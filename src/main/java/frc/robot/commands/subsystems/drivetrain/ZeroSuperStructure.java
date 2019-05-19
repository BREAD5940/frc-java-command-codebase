package frc.robot.commands.subsystems.drivetrain;

import org.team5940.pantry.exparimental.command.SendableCommandBase;

import frc.robot.SuperStructureConstants;
import frc.robot.lib.Logger;
import frc.robot.lib.obj.RoundRotation2d;
import frc.robot.states.ElevatorState;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class ZeroSuperStructure extends SendableCommandBase {
	String p;
	SuperStructure struc;
	SuperStructureState old;

	@Override
	public void schedule() {
		schedule(false);
	}

	public ZeroSuperStructure(String piece) {
		this.p = piece;
		// addRequirements(SuperStructure.getInstance());
		//		setInterruptible(false);
		struc = SuperStructure.getInstance();
		addRequirements(SuperStructure.getInstance());
		addRequirements(SuperStructure.getElevator());
		addRequirements(SuperStructure.getInstance().getElbow());
		addRequirements(SuperStructure.getInstance().getWrist());
		// Command comm = struc.getCurrentCommand();
		// if (comm != null)
		// comm.cancel();
	}

	@Override
	public void initialize() {

		old = struc.lastState;

		System.out.println("Zeroing encoder to " + p);
		if (p.equals("elevator")) {
			Logger.log("zeroing elevator");
			SuperStructure.getElevator().zeroEncoder();
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
			SuperStructure.getElevator().getMaster().getEncoder().resetPosition(
					SuperStructure.getElevator().getMaster().getModel().toNativeUnitPosition(
							(SuperStructureConstants.Elevator.top)

					).getValue());
			//			);
			old.getElevator().setHeight(SuperStructureConstants.Elevator.top);
		} else if (p.equals("maxWrist")) {
			SuperStructure.getInstance().getWrist().getMaster()
					.setSensorPosition(SuperStructureConstants.Wrist.kWristMax);
			old.getWrist().setAngle(SuperStructureConstants.Wrist.kWristMax);
		} else if (p.equals("minWrist")) {
			SuperStructure.getInstance().getWrist().getMaster()
					.setSensorPosition(SuperStructureConstants.Wrist.kWristMin);
			old.getWrist().setAngle(SuperStructureConstants.Wrist.kWristMin);
		} else if (p.equals("maxElbow")) {
			SuperStructure.getInstance().getWrist().getMaster()
					.setSensorPosition(SuperStructureConstants.Elbow.kElbowMax);
			old.getWrist().setAngle(SuperStructureConstants.Elbow.kElbowMax);
		} else if (p.equals("minElbow")) {
			SuperStructure.getInstance().getWrist().getMaster()
					.setSensorPosition(SuperStructureConstants.Elbow.kElbowMin);
			old.getWrist().setAngle(SuperStructureConstants.Elbow.kElbowMin);
		}
		// } else if (p.equals("topInnerElevator")) {
		// 	SuperStructure.getInstance().getElevator().getMaster().setSensorPosition(LengthKt.getInch(26));
		// 	old.getElevator().setHeight(LengthKt.getInch(26));
		// }
	}

	@Override
	public void execute() {

	}

	@Override
	public boolean isFinished() {
		return true;
	}

	//	@Override
	public void end(boolean interrupted) {
		struc.getDefaultCommand().cancel();
		System.out.println("Restarting the default command");
		struc.move(old);
		struc.getDefaultCommand().schedule();
	}

	//	@Override
	//	public void interrupted() {
	//
	//	}
}
