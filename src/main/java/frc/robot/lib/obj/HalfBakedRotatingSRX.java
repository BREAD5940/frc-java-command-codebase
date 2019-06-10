package frc.robot.lib.obj;

import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class HalfBakedRotatingSRX extends WPI_TalonSRX {

	double mModel; // ticks per rotation

	public HalfBakedRotatingSRX(int deviceNumber, double unitsPerRotation) {
		super(deviceNumber);
		this.mModel = unitsPerRotation;
	}

	public RoundRotation2d getRotation2d() {
		NativeUnit rawPos = NativeUnitKt
				.getNativeUnits(super.getSelectedSensorPosition());

		// first divide by count to get rotations
		double rotations = rawPos.div(mModel).getValue();

		// now construct a rotation2d from rotations
		return RoundRotation2d.fromRotations(rotations);
	}

	public void setSensorPosition(RoundRotation2d pos_) {
		int ticks = (int) Math.round(pos_.getRotations() * mModel);
		super.setSelectedSensorPosition(ticks);
	}

	public RoundRotation2d getSensorPosition() {
		int ticks = super.getSelectedSensorPosition();
		RoundRotation2d pos_ = RoundRotation2d.getDegree(ticks / mModel);
		return pos_;
	}

	public RoundRotation2d getError() {
		int ticks = super.getClosedLoopError();
		RoundRotation2d pos_ = RoundRotation2d.getDegree(ticks / mModel);
		return pos_;
	}

	public RoundRotation2d getRotation2dError() {
		int ticks = super.getClosedLoopError();
		RoundRotation2d toReturn = fromTicks(ticks);
		return toReturn;
	}

	public AngularVelocity getSensorVelocity() {
		int raw_ = super.getSelectedSensorVelocity();
		double rotPerSec = raw_ / mModel * 10;
		return new AngularVelocity(RoundRotation2d.fromRotations(rotPerSec),
				TimeUnitsKt.getSecond(0.1));
	}

	public RoundRotation2d fromTicks(int ticks) {
		double rotations = ticks / mModel;
		return RoundRotation2d.fromRotations(rotations);
	}

	public int getTicks(RoundRotation2d pos) {
		double rotation = pos.getRotations();
		int ticks = (int) (rotation * mModel);
		return ticks;
	}

	public int getTicks(Rotation2d pos) {
		return getTicks(RoundRotation2d.fromRotation2d(pos));
	}

	public RoundRotation2d getRotationsFromRaw(int rawPos) {
		double rotations = rawPos / mModel;
		return RoundRotation2d.fromRotations(rotations);
	}

	public void set(ControlMode mode, RoundRotation2d setpoint) {
		super.set(mode, getTicks(setpoint));
	}

	public void set(ControlMode mode, RoundRotation2d setpoint, DemandType type,
			double arg2) {
		if (mode == ControlMode.Position)
			super.set(mode, getTicks(setpoint), type, arg2);
		else {
			System.out
					.println("Cannot set to any other mode with RoundRotation2d. ur bad");
		}
	}

	public void setMotionCruiseVelocity(Velocity<Rotation2d> newVelocity) {
		super.configMotionCruiseVelocity(
				getTicks(RoundRotation2d.getRadian(newVelocity.getValue())));
	}

	//	public RoundRotation2d getVelocity() {
	//	}
}
