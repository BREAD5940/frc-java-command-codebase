package frc.robot.lib;

import edu.wpi.first.wpilibj.Notifier;

public abstract class Looper {
  private final double loopTime;
  Notifier notifier_;

  public Looper() {
    this(0.02);
  }

  public Looper(double loopTime){
    this.loopTime = loopTime;
    notifier_ = new Notifier(() -> execute());
    initilize();
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