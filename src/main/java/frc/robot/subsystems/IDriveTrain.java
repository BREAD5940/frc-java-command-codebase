package frc.robot.subsystems;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;

public interface IDriveTrain {

  public void setHighGear();

  public void setLowGear();

  public void stop();

  public void setVoltages(double left, double right);

  public void setClosedLoop(Velocity<Length> left, Velocity<Length> right);
  
	public void setCLosedLoop(Velocity<Length> left, Velocity<Length> right, double leftPercent, double rightPercent, boolean brakeMode);

  public void curvatureDrive(double linearPercent, double curvaturePercent, boolean isQuickTurn);

  // public Transmission getLeft();

  // public Transmission getRight();

  public void zeroEncoders();

}