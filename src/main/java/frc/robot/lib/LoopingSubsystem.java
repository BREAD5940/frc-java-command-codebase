package frc.robot.lib;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * A looping sybsystem is a subsystem that has components that need to run periodically,
 * but it can't be a command. Like the superstructure. This just extends Subsystem and 
 * functions the same as it, but this also requires the methods initilze(), execute(), and
 * end() to be immplemented
 * 
 * @author Matthew Morley
 */
public abstract class LoopingSubsystem extends Subsystem {
  private final double loopTime;
  private Notifier notifier_;

  public LoopingSubsystem() {
    this(0.02);
  }

  public LoopingSubsystem(double loopTime){
    this.loopTime = loopTime;
    notifier_ = new Notifier(() -> execute());
    initilize();
  }

  public Notifier getNotifier() {
    return notifier_;
  }

  public void startLooper() {
    notifier_.startPeriodic(loopTime);
  }

  public abstract void initilize();

  public abstract void execute();

  public abstract void end();

  public void stop() {
    end();
    notifier_.stop();
  }

}