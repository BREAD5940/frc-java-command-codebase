package frc.robot.commands.subsystems.drivetrain;

import org.team5940.pantry.exparimental.command.SendableCommandBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

/**
 * Default drivetrain command. This *should* be called as the default drivetrain
 * command and be overridden in autononmous (provided auto requires
 * drivetrain???) This command uses the Robot.m_oi to set the speed based on
 * xbox controller inputs, arcade style
 * 
 * @author Matthew Morley
 */
public class ArcadeDrive extends SendableCommandBase {

	// System.out.println("im an arcade drive command!");
	/**
	 * This command runs arcade drive as the default command for the drivetrain.
	 * This command will reserve the drivetrain.
	 */
	public ArcadeDrive() {
		addRequirements(Robot.drivetrain);
	}

	// drivetrain drivetrain = new drivetrain();

	// Called just before this Command runs the first time
	@Override
	public void initialize() {
		Robot.drivetrain.setNeutralMode(NeutralMode.Brake);
		Robot.drivetrain.getLeft().getMaster().configClosedloopRamp(0.2);
		Robot.drivetrain.getRight().getMaster().configClosedloopRamp(0.2);
		Robot.drivetrain.getLeft().getMaster().configOpenloopRamp(0.2);
		Robot.drivetrain.getRight().getMaster().configOpenloopRamp(0.2);
		Robot.drivetrain.arcadeDrive(0, 0);
		System.out.println("arcade drive command init");
		DriveTrain.getInstance().isFirstRun = true;
	}

	boolean wasPressed = false;

	// Called repeatedly when this Command is scheduled to run
	@Override
	public void execute() {
		// Robot.drivetrain.arcadeDrive(Robot.m_oi.getForwardAxis(),
		//   Robot.m_oi.getTurnAxis());

		// boolean isQuickTurn = (Math.abs(Robot.m_oi.getForwardAxis()) < 0.08);
		// boolean isQuickTurn = (Math.abs(Robot.m_oi.getPrimary().getRawAxis(0)) > 0.75);

		// boolean isHighGear = (DriveTrain.getInstance().getCachedGear() == Gear.HIGH);

		// if (isHighGear) {
		// 	Robot.drivetrain.arcadeDrive(Robot.m_oi.getForwardAxis() * 1,
		// 			Robot.m_oi.getTurnAxis(), true/*, isQuickTurn*/);
		// } else {
		// 	Robot.drivetrain.arcadeDrive(Robot.m_oi.getForwardAxis() * 1,
		// 			Robot.m_oi.getTurnAxis(), true/*, isQuickTurn*/);
		// }
		// DriveTrain.getInstance().closedLoopArcadeDrive(Robot.m_oi.getForwardAxis(), Robot.m_oi.getTurnAxis(), true);
		DriveTrain.getInstance().arcadeDrive(Robot.m_oi.getForwardAxis(), Robot.m_oi.getTurnAxis() * 0.8, true);

		// if((Robot.m_oi.getPrimary().getRawButton(1)) && (!wasPressed)) {
		// 	wasPressed = true;
		// 	var currentGear = Robot.getDrivetrainGear();
		// 	if(currentGear == Gear.LOW) {
		// 		DriveTrain.getInstance().setGear(Gear.HIGH);
		// 	} else {
		// 		DriveTrain.getInstance().setGear(Gear.LOW);
		// } 
		// } else if (wasPressed && !(Robot.m_oi.getPrimary().getRawButton(1))) {
		// 	wasPressed = false;
		// }

	}

	// Logger.log("forward command: " + Robot.m_oi.getForwardAxis());

	// Make this return true when this Command no longer needs to run execute()
	@Override
	public boolean isFinished() {
		// System.out.println("we aint done chief");
		return false;

	}

	// Called once after isFinished returns true
	@Override
	public void end(boolean interrupted) {
		Robot.drivetrain.arcadeDrive(0, 0);
		System.out.println("arcade end called");
		Robot.drivetrain.getLeft().getMaster().configClosedloopRamp(0.0);
		Robot.drivetrain.getRight().getMaster().configClosedloopRamp(0.0);
		Robot.drivetrain.getLeft().getMaster().configOpenloopRamp(0.0);
		Robot.drivetrain.getRight().getMaster().configOpenloopRamp(0.0);
	}

}
