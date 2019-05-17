package frc.robot.lib;

import org.team5940.pantry.experimental.command.SendableSubsystemBase;

public abstract class HalfBakedSubsystem extends Subsystem {

	public HalfBakedSubsystem(String string) {
		super(string);
	}

	public abstract void onDisable();

}
