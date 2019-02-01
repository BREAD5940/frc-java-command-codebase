package frc.math;

import java.text.DecimalFormat;

public class Pose2dWithCurvature /*implements IPose2d<Pose2dWithCurvature>, ICurvature<Pose2dWithCurvature>*/ {
    protected static final Pose2dWithCurvature kIdentity = new Pose2dWithCurvature();

    public static final Pose2dWithCurvature identity() {
        return kIdentity;
    }

    protected final Pose2d pose_;
    protected final double curvature_;
    protected final double dcurvature_ds_;

    public Pose2dWithCurvature() {
        pose_ = new Pose2d();
        curvature_ = 0.0;
        dcurvature_ds_ = 0.0;
    }

    public Pose2dWithCurvature(final Pose2d pose, double curvature) {
        pose_ = pose;
        curvature_ = curvature;
        dcurvature_ds_ = 0.0;
    }

    public Pose2dWithCurvature(final Pose2d pose, double curvature, double dcurvature_ds) {
        pose_ = pose;
        curvature_ = curvature;
        dcurvature_ds_ = dcurvature_ds;
    }

    public Pose2dWithCurvature(final Translation2d translation, final Rotation2d rotation, double curvature) {
        pose_ = new Pose2d(translation, rotation);
        curvature_ = curvature;
        dcurvature_ds_ = 0.0;
    }

    public Pose2dWithCurvature(final Translation2d translation, final Rotation2d rotation, double curvature, double dcurvature_ds) {
        pose_ = new Pose2d(translation, rotation);
        curvature_ = curvature;
        dcurvature_ds_ = dcurvature_ds;
    }

    
    public final Pose2d getPose() {
        return pose_;
    }

    
    public Pose2dWithCurvature transformBy(Pose2d transform) {
        return new Pose2dWithCurvature(getPose().transformBy(transform), getCurvature(), getDCurvatureDs());
    }

    
    public Pose2dWithCurvature mirror() {
        return new Pose2dWithCurvature(getPose().mirror().getPose(), -getCurvature(), -getDCurvatureDs());
    }

    
    public double getCurvature() {
        return curvature_;
    }

    
    public double getDCurvatureDs() {
        return dcurvature_ds_;
    }

    
    public final Translation2d getTranslation() {
        return getPose().getTranslation();
    }

    
    public final Rotation2d getRotation() {
        return getPose().getRotation();
    }

    
    public Pose2dWithCurvature interpolate(final Pose2dWithCurvature other, double x) {
        return new Pose2dWithCurvature(getPose().interpolate(other.getPose(), x),
                Util.interpolate(getCurvature(), other.getCurvature(), x),
                Util.interpolate(getDCurvatureDs(), other.getDCurvatureDs(), x));
    }

    
    public double distance(final Pose2dWithCurvature other) {
        return getPose().distance(other.getPose());
    }

    
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof Pose2dWithCurvature)) return false;
        Pose2dWithCurvature p2dwc = (Pose2dWithCurvature) other;
        return getPose().equals(p2dwc.getPose()) && Util.epsilonEquals(getCurvature(), p2dwc.getCurvature()) && Util.epsilonEquals(getDCurvatureDs(), p2dwc.getDCurvatureDs());
    }

    
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return getPose().toString() + ", curvature: " + fmt.format(getCurvature()) + ", dcurvature_ds: " + fmt.format(getDCurvatureDs());
    }

    
    public String toCSV() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return getPose().toCSV() + "," + fmt.format(getCurvature()) + "," + fmt.format(getDCurvatureDs());
    }
}
