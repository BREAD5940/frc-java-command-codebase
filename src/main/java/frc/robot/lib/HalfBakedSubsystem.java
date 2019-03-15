package frc.robot.lib;

import edu.wpi.first.wpilibj.command.Subsystem;

public abstract class HalfBakedSubsystem extends Subsystem {

	public HalfBakedSubsystem(String string) {
		super(string);
	}

	public abstract void onDisable();

}
