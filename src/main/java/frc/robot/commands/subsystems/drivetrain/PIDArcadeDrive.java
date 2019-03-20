/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.subsystems.drivetrain;

import java.util.TreeMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team254.lib.physics.DifferentialDrive.ChassisState;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.lib.InterpolatableLut;
import frc.robot.lib.InterpolatableLutEntry;
import frc.robot.lib.motion.Util;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;

public class PIDArcadeDrive extends Command {

	private ChassisState mCachedChassisState;
	public boolean isFirstRun = true;

  private InterpolatableLut highGearAngularLUT, lowGearAngularLUT;

	private boolean squareInputs;

	public PIDArcadeDrive(boolean square) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(DriveTrain.getInstance());
    this.squareInputs = square;
    
    var highGearMap = new TreeMap<Double, InterpolatableLutEntry>();
    highGearMap.put(Double.valueOf(0), new InterpolatableLutEntry(8));
    highGearMap.put(Double.valueOf(5), new InterpolatableLutEntry(8));
    highGearMap.put(Double.valueOf(15), new InterpolatableLutEntry(16));
    highGearAngularLUT = new InterpolatableLut(highGearMap);

    var lowGearMap = new TreeMap<Double, InterpolatableLutEntry>();
    lowGearMap.put(Double.valueOf(0), new InterpolatableLutEntry(10));
    lowGearMap.put(Double.valueOf(5), new InterpolatableLutEntry(10));
    lowGearMap.put(Double.valueOf(10), new InterpolatableLutEntry(15));
    lowGearAngularLUT = new InterpolatableLut(lowGearMap);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		DriveTrain.getInstance().setNeutralMode(NeutralMode.Coast);
		DriveTrain.getInstance().getLeft().getMaster().configClosedloopRamp(0.12);
		DriveTrain.getInstance().getRight().getMaster().configClosedloopRamp(0.12);
		DriveTrain.getInstance().getLeft().getMaster().configOpenloopRamp(0.12);
		DriveTrain.getInstance().getRight().getMaster().configOpenloopRamp(0.12);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {

		var linearPercent = Robot.m_oi.getForwardAxis();
		var rotationPercent = Robot.m_oi.getTurnAxis();

		linearPercent = Util.limit(linearPercent, 1);
		linearPercent = Util.deadband(linearPercent, 0.07);

		rotationPercent = Util.limit(rotationPercent, 1);
		rotationPercent = Util.deadband(rotationPercent, 0.1);

		// Square the inputs (while preserving the sign) to increase fine control
		// while permitting full power.
		if (squareInputs) {
			linearPercent = Math.copySign(linearPercent * linearPercent, linearPercent);
			rotationPercent = Math.copySign(rotationPercent * rotationPercent, rotationPercent);
		}

		double leftMotorOutput;
		double rightMotorOutput;

		double maxInput = Math.copySign(Math.max(Math.abs(linearPercent), Math.abs(rotationPercent)), linearPercent);

		if (linearPercent >= 0.0) {
			// First quadrant, else second quadrant
			if (rotationPercent >= 0.0) {
				leftMotorOutput = maxInput;
				rightMotorOutput = linearPercent - rotationPercent;
			} else {
				leftMotorOutput = linearPercent + rotationPercent;
				rightMotorOutput = maxInput;
			}
		} else {
			// Third quadrant, else fourth quadrant
			if (rotationPercent >= 0.0) {
				leftMotorOutput = linearPercent + rotationPercent;
				rightMotorOutput = maxInput;
			} else {
				leftMotorOutput = maxInput;
				rightMotorOutput = linearPercent - rotationPercent;
			}
		}
		// Logger.log("Linear input " + linearPercent + " turn input " +
		// rotationPercent);
		// Logger.log("left motor output " + leftMotorOutput + " right motor output " +
		// rightMotorOutput);

		boolean isHighGear = DriveTrain.getInstance().getCachedGear() == Gear.HIGH;

		final double lowGearForward = Util.toMeters(7.5);
		final double highGearForward = Util.toMeters(11);
		double lowGearTurn = Util.toMeters(12);
		double highGearTurn = Util.toMeters(18);

		final double maxAccelLinearLow = Util.toMeters(12);
		final double maxAccelLinearHigh = Util.toMeters(18);
		final double maxAccelAngularLow = Util.toMeters(12);
		final double maxAccelAngularHigh = Util.toMeters(13);

		double forwardSpeed = linearPercent * ((isHighGear) ? highGearForward : lowGearForward);

    // double turnSpeed = 
    // interpolate turn speed based on forward speed

    double turnSpeed = -1 * rotationPercent * ((isHighGear) ? highGearTurn : lowGearTurn);

		// forwardSpeed = Util.limit(forwardSpeed, forwardSpeed-(maxAccelLinearLow * Robot.mPeriod), forwardSpeed+(maxAccelLinearLow * Robot.mPeriod))

		ChassisState mVelocity = new ChassisState(forwardSpeed, turnSpeed);
		if (isFirstRun) {
			mCachedChassisState = new ChassisState();
			isFirstRun = false;
		}

		if (isHighGear) {
      // limit limear acceleration
			mVelocity.setLinear(Util.limit(mVelocity.getLinear(), mCachedChassisState.getLinear() - maxAccelLinearHigh / Robot.mPeriod,
					mCachedChassisState.getLinear() + maxAccelLinearHigh / Robot.mPeriod));

      // limit the angular acceleration
			mVelocity.setAngular(Util.limit(mVelocity.getAngular(), mCachedChassisState.getAngular() - maxAccelAngularHigh / Robot.mPeriod,
        mCachedChassisState.getAngular() + maxAccelAngularHigh / Robot.mPeriod));
      
      mVelocity.setLinear(Util.limit(mVelocity.getLinear(), highGearForward)); // constrain linear velocity just in case
      // calculate the max turn speed based on the linear speed
      highGearTurn = highGearAngularLUT.interpolate(Util.toFeet(mVelocity.getLinear()));
			mVelocity.setAngular(Util.limit(mVelocity.getAngular(), highGearTurn)); // limit angular velocity based on calculated max speed
		} else {
			mVelocity.setLinear(Util.limit(mVelocity.getLinear(), mCachedChassisState.getLinear() - maxAccelLinearLow / Robot.mPeriod,
					mCachedChassisState.getLinear() + maxAccelLinearLow / Robot.mPeriod));
      mVelocity.setLinear(Util.limit(mVelocity.getLinear(), lowGearForward));
      lowGearTurn = lowGearAngularLUT.interpolate(Util.toFeet(mVelocity.getLinear()));
			mVelocity.setAngular(Util.limit(mVelocity.getAngular(), lowGearTurn));
		}

		// ChassisState mAcceleration = new ChassisState(
				// (mVelocity.getLinear() - mCachedChassisState.getLinear()) / Robot.mPeriod,
				// (mVelocity.getAngular() - mCachedChassisState.getAngular()) / Robot.mPeriod);

		// System.out.println("mVelocity: " + mVelocity.getLinear() + " mAccel: " + mAcceleration.getLinear());

    // DriveTrain.getInstance().setOutputFromDynamics(mVelocity, mAcceleration);
    DriveTrain.getInstance().setOutputFromKinematics(mVelocity);

    // var wheelVelocities = DriveTrain.getInstance().getDifferentialDrive().solveInverseKinematics(mVelocity);
		// var feedForwardVoltages = DriveTrain.getInstance().getDifferentialDrive().getVoltagesFromkV(wheelVelocities);
		
    // DriveTrain.getInstance().tankDrive(feedForwardVoltages.getLeft() / 12, feedForwardVoltages.getRight() / 12);


		mCachedChassisState = mVelocity;
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
    DriveTrain.getInstance().stop();
  }

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
    DriveTrain.getInstance().stop();
  }
}
