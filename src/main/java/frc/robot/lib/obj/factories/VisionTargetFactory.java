package frc.robot.lib.obj.factories;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;

import frc.robot.lib.obj.VisionTarget;

public class VisionTargetFactory {

	private static final Length kHatchTapeHeight = LengthKt.getInch(12); //distance from floor to bottom of tape TODO I don't think we need this

	private static final Length kRocketCargoTapeHeight = LengthKt.getInch(20); //distance from floor to bottom of tape TODO I don't think we need this

	public static VisionTarget getRocketCargoDualTarget() {
		return new VisionTarget(VisionTarget.kDualVisionTapeShape, kRocketCargoTapeHeight);
	}

	public static VisionTarget getHatchDualTarget() {
		return new VisionTarget(VisionTarget.kDualVisionTapeShape, kHatchTapeHeight);
	}

	public static VisionTarget getRocketCargoSingleTarget() {
		return new VisionTarget(VisionTarget.kSingleVisionTapeShape, kRocketCargoTapeHeight);
	}

	public static VisionTarget getHatchSingleTarget() {
		return new VisionTarget(VisionTarget.kSingleVisionTapeShape, kHatchTapeHeight);
	}

}
