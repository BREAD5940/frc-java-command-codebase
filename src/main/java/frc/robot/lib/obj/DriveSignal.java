package frc.robot.lib.obj;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;

/**
 * A drivetrain command consisting of the left, right motor settings and whether
 * the brake mode is enabled.
 */
public class DriveSignal {
    protected Velocity<Length> mLeftMotor;
    protected Velocity<Length> mRightMotor;
    protected double mLeftPercent;
    protected double mRightPercent;
    protected boolean mBrakeMode;

    public DriveSignal(Velocity<Length> left, Velocity<Length> right) {
        this(left, right, false);
    }

    public DriveSignal(double leftPercent, double rightPercent) {
        this(leftPercent, rightPercent, false);
    }

    public DriveSignal(double leftPercent, double rightPercent, boolean mBrakeMode) {
        this(zeroSpeed, zeroSpeed, leftPercent, rightPercent, false);
    }

    public DriveSignal(Velocity<Length> left, Velocity<Length> right, boolean brakeMode) {
        this(left, right, 0, 0, false);
    }

    public DriveSignal(Velocity<Length> left, Velocity<Length> right, double leftPercent, double rightPercent, boolean brakeMode) {
        mLeftMotor = left;
        mRightMotor = right;
        mLeftPercent = leftPercent;
        mRightPercent = rightPercent;
        mBrakeMode = brakeMode;
    }

    public static DriveSignal NEUTRAL = new DriveSignal(VelocityKt.getVelocity(LengthKt.getFeet(0)), VelocityKt.getVelocity(LengthKt.getFeet(0f)));
    public static DriveSignal BRAKE = new DriveSignal(VelocityKt.getVelocity(LengthKt.getFeet(0f)), VelocityKt.getVelocity(LengthKt.getFeet(0f)), true);
    private static final Velocity<Length> zeroSpeed = VelocityKt.getVelocity(LengthKt.getFeet(0f));

    public Velocity<Length> getLeft() {
        return mLeftMotor;
    }

    public Velocity<Length> getRight() {
        return mRightMotor;
    }

    public double getLeftPercent() {
        return mLeftPercent;
    }

    public double getRightPercent() {
        return mRightPercent;
    }

    public boolean getBrakeMode() {
        return mBrakeMode;
    }

    @Override
    public String toString() {
        return "L: " + mLeftMotor + ", R: " + mRightMotor + (mBrakeMode ? ", BRAKE" : "");
    }
}