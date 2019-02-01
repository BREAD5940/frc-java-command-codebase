package frc.math;

import frc.math.Util;

import java.text.DecimalFormat;

/**
 * A movement along an arc at constant curvature and velocity. We can use ideas from "differential calculus" to create
 * new RigidTransform2d's from a Twist2d and visa versa.
 * <p>
 * A Twist can be used to represent a difference between two poses, a velocity, an acceleration, etc.
 */
public class Twist2d {
    protected static final Twist2d kIdentity = new Twist2d(0.0, 0.0, 0.0);

    public static final Twist2d identity() {
        return kIdentity;
    }

    public final double dx;
    public final double dy;
    public final double dtheta; // Radians!

    /**
     * Instantiate a new Twist2d using dx, dy and dtheta components.
     * This is the only constructor, so have fun using something else.
     * 
     * @param dx (forward/back)
     * @param dy (left/right)
     * @param dtheta (in radians!)
     */
    public Twist2d(double dx, double dy, double dtheta) {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

    /**
     * Scale this' Twist2d by a scaler (hah, see? Vocab words :D)
     * 
     * @param scale
     * @return A new Twist2d scaled by scale
     */
    public Twist2d scaled(double scale) {
        return new Twist2d(dx * scale, dy * scale, dtheta * scale);
    }

    /**
     * return the Norm (i.e. distance forumula) of x and y displacement.
     * If we don't turn much (drive straight lol), dy will be zero so we
     * be lazy. otherwise, we just return the distance forumula straight
     * line distance between the two points.
     * 
     * @return norm (distance) of the Twist2d
     */
    public double norm() {
        // Common case of dy == 0
        if (dy == 0.0)
            return Math.abs(dx);
        return Math.hypot(dx, dy);
    }

    /**
     * Get the curvature of this' twist2d. This method takes no arguments
     * and will return zero if (1) the angle is absurdly small and (2) the
     * {@link norm} of this twist is absurdly small.
     * @return the curvature of the twist2d
     */
    public double curvature() {
        if (Math.abs(dtheta) < Util.kEpsilon && norm() < Util.kEpsilon)
            return 0.0;
        return dtheta / norm();
    }

    @Override
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return "(" + fmt.format(dx) + "," + fmt.format(dy) + "," + fmt.format(Math.toDegrees(dtheta)) + " deg)";
    }
}