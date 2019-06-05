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
		requires(SuperStructure.getElevator());
		setTimeout(kDefaultTimeout);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {

		Elevator elev = SuperStructure.getElevator();
		elev.getMaster().configMotionAcceleration((int) (600 * 9 * 1.75));
		elev.getMaster().configMotionCruiseVelocity(4000); // about 3500 theoretical max
		elev.getMaster().configMotionSCurveStrength(4);
		elev.getMaster().config_kP(3, 0.05 * 9, 0);
		elev.getMaster().config_kI(3, 0.0, 0);
		elev.getMaster().config_kD(3, 0.0, 0);
		elev.getMaster().config_kF(3, 0.24 * (500 / 400), 0);
		elev.getMaster().selectProfileSlot(3, 0);
		// elev.getMaster().configClosedloopRamp(0.1);

		Length mTarget = LengthKt.getFeet(1);
		double volts = Elevator.getVoltage(new SuperStructureState(new ElevatorState(mTarget), new RotatingArmState(), new RotatingArmState()));
		// SuperStructure.getElevator().requestClosedLoop(ControlMode.MotionMagic, mTarget, DemandType.ArbitraryFeedForward, volts);
		// SuperStructure.getElevator().getMaster().set(ControlMode.Velocity,
		// 		500, DemandType.ArbitraryFeedForward, volts);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		mCurrent = SuperStructure.getInstance().getCurrentState();
		// System.out.printf("Forward soft lim: %s rev soft lim: %s", SuperStructure.getElevator().getMaster().softlimit )
		Length mTarget = LengthKt.getFeet(1);
		double volts = Elevator.getVoltage(new SuperStructureState(new ElevatorState(mTarget), new RotatingArmState(), new RotatingArmState())) / 12;
		// SuperStructure.getElevator().requestClosedLoop(ControlMode.MotionMagic, mTarget, DemandType.ArbitraryFeedForward, volts);
		double targetTic = (int) SuperStructure.getElevator().getModel().toNativeUnitPosition(mTarget).getValue();
		// System.out.println(targetTic);
		SuperStructure.getElevator().getMaster().set(ControlMode.MotionMagic,
				LengthKt.getInch(12), DemandType.ArbitraryFeedForward, volts);

		// System.out.println(SuperStructure.getElevator().getMaster().getSelectedSensorVelocity() / (391.13918814264197718560873686429) * 0.23);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
		// return SuperStructure.getElevator().isWithinTolerence(mCurrent) || isTimedOut();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		SuperStructure.elevator.setGear(Elevator.kDefaultGear);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {}
}
