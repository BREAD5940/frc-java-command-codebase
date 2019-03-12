package frc.robot.lib;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

import frc.robot.lib.obj.RoundRotation2d;

public class RotatingSparkMax extends CANSparkMax {

	// NativeUnitLengthModel model;
	CANEncoder neoEncoder;
	double reduction;
	int m_currentPIDSlot = 0;
	CANPIDController controller;

	public RotatingSparkMax(int port, MotorType type, double reduction) {
		super(port, type);
		neoEncoder = new CANEncoder(this);
		this.reduction = reduction;
		controller = super.getPIDController();
	}

	public RoundRotation2d getRotation() {
		var rotations = neoEncoder.getPosition();
		return RoundRotation2d.fromRotations(rotations);
	}

	public void setClosedLoopGains(int slot, PIDSettings settings) {
		controller.setP(settings.kp, slot);
		controller.setI(settings.ki, slot);
		controller.setD(settings.kd, slot);
		controller.setFF(settings.kf, slot);
		m_currentPIDSlot = slot;
	}

}
