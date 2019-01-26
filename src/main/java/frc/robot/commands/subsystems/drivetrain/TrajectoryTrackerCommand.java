package frc.robot.commands.subsystems.drivetrain;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

import org.ghrobotics.lib.debug.LiveDashboard;
import org.ghrobotics.lib.localization.Localization;
import org.ghrobotics.lib.mathematics.twodim.control.TrajectoryTracker;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d;
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2dWithCurvature;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedEntry;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TimedTrajectory;
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.TrajectorySamplePoint;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerDriveBase;
import org.ghrobotics.lib.subsystems.drive.TrajectoryTrackerOutput;

import java.util.function.Supplier;

// @SuppressWarnings({"WeakerAccess", "unused"})
public class TrajectoryTrackerCommand extends Command {
    private TrajectoryTracker trajectoryTracker;
    private Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySource;
    private DriveTrain driveBase;
    private Localization localization;
    private boolean reset;
    private TrajectoryTrackerOutput output;

    public TrajectoryTrackerCommand(DriveTrain driveBase, Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySource){
        this(driveBase, trajectorySource, false);
    }

    public TrajectoryTrackerCommand(DriveTrain driveBase, Supplier<TimedTrajectory<Pose2dWithCurvature>> trajectorySource, boolean reset){
        requires(driveBase);
        this.driveBase = driveBase;
        this.trajectoryTracker = driveBase.getTrajectoryTracker();
        this.trajectorySource = trajectorySource;
        this.reset = reset;
    }

    @Override
    protected void initialize(){
        trajectoryTracker.reset(trajectorySource.get());

        if(reset) {
            localization.reset(trajectorySource.get().getFirstState().component1().getPose());
        }

        LiveDashboard.INSTANCE.setFollowingPath(true);
    }

    @Override
    protected void execute(){
        output = trajectoryTracker.nextState(driveBase.getRobotPosition(), TimeUnitsKt.getMillisecond(System.currentTimeMillis()));

        TrajectorySamplePoint<TimedEntry<Pose2dWithCurvature>> referencePoint = trajectoryTracker.getReferencePoint();
        if(referencePoint != null){
            Pose2d referencePose = referencePoint.getState().getState().getPose();

            LiveDashboard.INSTANCE.setPathX(referencePose.getTranslation().getX().getFeet());
            LiveDashboard.INSTANCE.setPathY(referencePose.getTranslation().getY().getFeet());
            LiveDashboard.INSTANCE.setPathHeading(referencePose.getRotation().getRadian());
        }
    }

    @Override
    protected void end(){
        driveBase.stop();
        LiveDashboard.INSTANCE.setFollowingPath(false);
    }

    @Override
    protected boolean isFinished() {
        return trajectoryTracker.isFinished();
    }
}