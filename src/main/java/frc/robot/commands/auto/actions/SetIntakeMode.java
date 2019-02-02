package frc.robot.commands.auto.actions;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.auto.AutoMotion.HeldPiece;

  /**
   * auto_Intake is a basic auto action. It activates the intake based on an inputted
   * demand and runtime
   */
public class SetIntakeMode extends Command {
  HeldPiece intakeType;
  boolean isDrop=false;

  public SetIntakeMode(HeldPiece iType){
    this.intakeType=iType;
    this.isDrop=false;
  }

  public SetIntakeMode(){
    this.isDrop=true;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(isDrop){
      // TODO code for actuating wrist to cargo side pointed down here
    }else{
      switch(intakeType){
        case CARGO:
          // TODO code for actuating wrist to cargo side here?
          break;
        case HATCH:
          // TODO code for actuating wrist to hatch side here?
          break;
        case NONE:
          // This should actually never happen, but if it does it just doesn't do anything
          break;
      }
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    // Don't have to do anything, just wait for the timeout to trigger
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true; // TODO change this?
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
