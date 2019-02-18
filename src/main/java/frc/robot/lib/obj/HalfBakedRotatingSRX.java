package frc.robot.lib.obj;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnit;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitRotationModel;

public class HalfBakedRotatingSRX extends WPI_TalonSRX {

  NativeUnitRotationModel mModel;

  public HalfBakedRotatingSRX(int deviceNumber, NativeUnitRotationModel mModel_) {
    super(deviceNumber);
    this.mModel = mModel_;
  }

  public Rotation2d getRotation2d() {
    NativeUnit rawPos = NativeUnitKt.getNativeUnits(super.getSelectedSensorPosition());
    return mModel.fromNativeUnitPosition(rawPos);
  }

}