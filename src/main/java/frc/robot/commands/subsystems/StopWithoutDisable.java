package frc.robot.commands.subsystems;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.superstructure.SuperStructure;

public class StopWithoutDisable extends Command {
	boolean drivetrain;

	public StopWithoutDisable(boolean drivetrain) {
		if (drivetrain) {
			requires(DriveTrain.getInstance());
		} else {
			requires(SuperStructure.getInstance());
		}
		this.drivetrain = drivetrain;
	}

	@Override
	public void initialize() {
		if (drivetrain) {
			DriveTrain.getInstance().stop();
			//FIXME this won't make the robot do an Inertia and yeet itself over, right?
		} else {
			SuperStructure.getInstance().move(new SuperStructureState());
		}
	}

	@Override
	public void execute() {
		if (drivetrain) {
			DriveTrain.getInstance().getDefaultCommand().cancel();
			DriveTrain.getInstance().getCurrentCommand().cancel();
		} else {
			SuperStructure.getInstance().getDefaultCommand().cancel();
			SuperStructure.getInstance().getCurrentCommand().cancel();
		}
	}

	@Override
	public boolean isFinished() {
		return true;
	}

	@Override
	public void end() {}

	@Override
	public void interrupted() {

	}
}
