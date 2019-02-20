package frc.robot.lib.obj.factories;

import java.util.Arrays;
import java.util.List;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import frc.robot.lib.obj.VisionTarget;

public class VisionTargetFactory {

  private static final Length kHatchTapeHeight = LengthKt.getInch(12); //distance from floor to bottom of tape TODO I don't think we need this

  private static final Length kRocketCargoTapeHeight = LengthKt.getInch(20); //distance from floor to bottom of tape TODO I don't think we need this

  public static VisionTarget getRocketCargoTarget() {
    return new VisionTarget(kRocketCargoTapeHeight);
  }

  public static VisionTarget getHatchTarget() {
    return new VisionTarget(kHatchTapeHeight);
  }
  
}