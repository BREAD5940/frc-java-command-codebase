package frc.robot.lib.obj;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;

public class HalfBakedRotatingSRX extends WPI_TalonSRX {

  double mModel;

  public HalfBakedRotatingSRX(int deviceNumber, double unitsPerRotation) {
    super(deviceNumber);
    this.mModel = unitsPerRotation;
  }

  public RoundRotation2d getRotation2d() {
    NativeUnit rawPos = NativeUnitKt.getNativeUnits(super.getSelectedSensorPosition());

    // first divide by count to get rotations
    double rotations = rawPos.div(mModel).getValue();

    // now construct a rotation2d from rotations
    return RoundRotation2d.fromRotations(rotations);
  }

  public void setSensorPosition(RoundRotation2d pos_) {
    int ticks = (int) Math.round(pos_.getRotations() * mModel);
    super.setSelectedSensorPosition(ticks);
  }

  public AngularVelocity getSensorVelocity() {
    int raw_ = super.getSelectedSensorVelocity();
    double rotPerSec = raw_ / mModel * 10;
    return new AngularVelocity(RoundRotation2d.fromRotations(rotPerSec), TimeUnitsKt.getSecond(0.1));
  }

  // public CompatVelocity<RoundRotation2d> getSensorVelocity(){
  //   return 
  // }
}
