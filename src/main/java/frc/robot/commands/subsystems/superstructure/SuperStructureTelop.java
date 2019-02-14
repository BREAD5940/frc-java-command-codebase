package frc.robot.commands.subsystems.superstructure;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.wrappers.ctre.FalconSRX;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.SuperStructure;

public class SuperStructureTelop extends Command {
	private SuperStructure superStructure;

	/** Run the superstructure (elevator for now) during telop using an xbox
	  * joystick. 
	  * @param struc the superstructure object
	  **/
	public SuperStructureTelop(SuperStructure struc) {
		// Use requires() here to declare subsystem dependencies
		requires(struc);
		this.superStructure = struc;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
    Length delta = LengthKt.getInch(Robot.m_oi.getElevatorAxis() * 0.08);
    Elevator elev = superStructure.getElevator(); // THIS LINE THROWS A HEKKING NULL PIONTER
    /*
     0.020000s 
 arcade drive command init 
ERROR  1  Unhandled exception: java.lang.NullPointerException  frc.robot.commands.subsystems.superstructure.SuperStructureTelop.execute(SuperStructureTelop.java:28) 
 Error at frc.robot.commands.subsystems.superstructure.SuperStructureTelop.execute(SuperStructureTelop.java:28): Unhandled exception: java.lang.NullPointerException 
 	at frc.robot.commands.subsystems.superstructure.SuperStructureTelop.execute(SuperStructureTelop.java:28) 
 	at edu.wpi.first.wpilibj.command.Command.run(Command.java:292) 
 	at edu.wpi.first.wpilibj.command.Scheduler.run(Scheduler.java:224) 
 	at frc.robot.Robot.teleopPeriodic(Robot.java:231) 
 	at edu.wpi.first.wpilibj.IterativeRobotBase.loopFunc(IterativeRobotBase.java:240) 
 	at edu.wpi.first.wpilibj.TimedRobot.startCompetition(TimedRobot.java:81) 
 	at edu.wpi.first.wpilibj.RobotBase.startRobot(RobotBase.java:263) 
 	at frc.robot.Main.main(Main.java:26) 
  
Warning  1  Loop time of 0.02s overrun
  edu.wpi.first.wpilibj.IterativeRobotBase.printLoopOverrunMessage(IterativeRobotBase.java:273) 
 Warning at edu.wpi.first
 */
    FalconSRX<Length> talon = elev.getMaster();
    Length current = talon.getSensorPosition();
    Length new_s = current.plus(delta);
    superStructure.moveSuperstructureElevator(new_s);
    System.out.println("target height: " + new_s);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {}
}
