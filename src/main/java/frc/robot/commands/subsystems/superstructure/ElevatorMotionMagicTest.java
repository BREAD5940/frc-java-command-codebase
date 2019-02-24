package frc.robot.commands.subsystems.superstructure;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.states.ElevatorState;
import frc.robot.states.SuperStructureState;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.RotatingJoint.RotatingArmState;
import frc.robot.subsystems.superstructure.SuperStructure;

public class ElevatorMotionMagicTest extends Command {

	SuperStructureState mCurrent;
	private final double kDefaultTimeout = 5;

	public ElevatorMotionMagicTest() {
		requires(SuperStructure.getInstance());
		setTimeout(kDefaultTimeout);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Length mTarget = LengthKt.getFeet(1);
		double volts = Elevator.getVoltage(new SuperStructureState(new ElevatorState(mTarget), new RotatingArmState(), new RotatingArmState()));
		// SuperStructure.getElevator().requestClosedLoop(ControlMode.MotionMagic, mTarget, DemandType.ArbitraryFeedForward, volts);
		SuperStructure.getElevator().getMaster().set(ControlMode.MotionMagic,
				(int) SuperStructure.getElevator().getModel().toNativeUnitPosition(mTarget).getValue(), DemandType.ArbitraryFeedForward, volts);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		mCurrent = SuperStructure.getInstance().updateState();
		// System.out.printf("Forward soft lim: %s rev soft lim: %s", SuperStructure.getElevator().getMaster().softlimit )
		Length mTarget = LengthKt.getFeet(1);
		double volts = Elevator.getVoltage(new SuperStructureState(new ElevatorState(mTarget), new RotatingArmState(), new RotatingArmState()));
		// SuperStructure.getElevator().requestClosedLoop(ControlMode.MotionMagic, mTarget, DemandType.ArbitraryFeedForward, volts);
		double targetTic = (int) SuperStructure.getElevator().getModel().toNativeUnitPosition(mTarget).getValue();
		System.out.println(targetTic);
		SuperStructure.getElevator().getMaster().set(ControlMode.MotionMagic,
				targetTic, DemandType.ArbitraryFeedForward, volts);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
		// return SuperStructure.getElevator().isWithinTolerence(mCurrent) || isTimedOut();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {}
}
