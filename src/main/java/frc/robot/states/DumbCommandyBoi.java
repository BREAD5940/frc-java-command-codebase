package frc.robot.states;

import org.ghrobotics.lib.mathematics.units.Length;

import frc.robot.SuperStructureConstants;
import frc.robot.lib.obj.RoundRotation2d;

public class DumbCommandyBoi {
	public Length height = SuperStructureConstants.Elevator.bottom;
	public RoundRotation2d wristAngle = SuperStructureConstants.Elbow.kElbowMin;

	public boolean openLoopElevator = false;
	public double openLoopElevatorPercent = 0.0;

	public boolean elevatorLowGear = false;
	public boolean deployForklift = false;
}
