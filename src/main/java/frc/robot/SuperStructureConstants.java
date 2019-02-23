package frc.robot;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Mass;
import org.ghrobotics.lib.mathematics.units.MassKt;

import frc.robot.lib.obj.RoundRotation2d;

public class SuperStructureConstants {
	public static class Wrist {
		public static final RoundRotation2d kWristMin = RoundRotation2d.getDegree(-45); // relative
		public static final RoundRotation2d kWristMax = RoundRotation2d.getDegree(90); // relative
		public static final Length intakeOut = LengthKt.getInch(19); //FIXME check
		public static final Length intakeDown = LengthKt.getInch(6); //FIXME check
		public static final Length intakeUp = LengthKt.getInch(12); //FIXME check
	}

	public static class Elbow {
		public static final RoundRotation2d kElbowMin = RoundRotation2d.getDegree(-180); // absolute
		public static final RoundRotation2d kElbowMax = RoundRotation2d.getDegree(15); // absolute
		public static final Length carriageToIntake = LengthKt.getInch(12); //FIXME verify
		public static final RoundRotation2d kStowedAngle = RoundRotation2d.getDegree(-80); //FIXME completely arb
		public static final RoundRotation2d kClearFirstStageMinElbowAngle = RoundRotation2d.getDegree(-70); //FIXME

	}

	public static class Elevator {
		public static final Length bottom = LengthKt.getInch(RobotConfig.elevator.elevator_minimum_height.getInch() + 0.5);
		public static final Length top = RobotConfig.elevator.elevator_maximum_height;
		public static final Length crossbarBottom = LengthKt.getInch(35); //FIXME verify
		public static final Length crossbarWidth = LengthKt.getInch(4); //FIXME verify
		public static final Length kElevatorLongRaiseDistance = LengthKt.getInch(27);//FIXME mostly arb.
		public static final Length kClearFirstStageMaxHeight = crossbarBottom; //yay its redundant
		public static final Length kElevatorApproachingThreshold = LengthKt.getInch(2); //FIXME

	}


	public static final Mass kHatchMass = MassKt.getLb(2.4); // FIXME check mass
	public static final Mass kCargoMass = MassKt.getLb(1); // FIXME check mass
}
