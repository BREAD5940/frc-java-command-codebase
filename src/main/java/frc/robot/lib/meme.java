package frc.robot.lib;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class meme  extends Command {

    Solenoid solenoid = new Solenoid(0);

    public meme() {
        solenoid.get();
        solenoid.set(true);
    }

    @Override
    public void initSendable(SendableBuilder builder) {

        builder.addboo

        super.initSendable(builder);
    }
}
