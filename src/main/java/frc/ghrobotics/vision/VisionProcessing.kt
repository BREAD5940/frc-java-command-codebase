package frc.ghrobotics.vision

import com.google.gson.JsonObject
import frc.robot.subsystems.DriveTrain
//import org.ghrobotics.frc2019.Constants
//import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
//import org.ghrobotics.frc2019.subsystems.elevator.ElevatorSubsystem
import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.inch
import org.ghrobotics.lib.mathematics.units.second

object VisionProcessing {

    fun processData(visionData: VisionData) {

        // TODO implement this
//        if (visionData.isFront && ElevatorSubsystem.position in Constants.kElevatorBlockingCameraRange) {
//            // Vision cannot see through the carriage
//            return
//        }

        val robotPose = DriveTrain.getInstance().localization[visionData.timestamp.second]

        val samples = visionData.targets
                .asSequence()
                .mapNotNull { // make sure the return of processReflectiveTape is non-null
                    processReflectiveTape(
                            it,
                            if (visionData.isFront) kCenterToFrontCamera else kCenterToBackCamera
                    )
                }
                .filter { // make sure that the vision target Pose2d is outside the robot
                    // We cannot be the vision target :)
                    it.translation.x.absoluteValue > kRobotLength / 2.0 - 5.inch
                            || it.translation.y.absoluteValue > kRobotWidth / 2.0
                }
                .map { robotPose + it } // sum the robotpose and the targetpose to get a global pose
                .toList() // make it into a List

        TargetTracker.addSamples(
            visionData.timestamp, samples
        )
    }

    private fun processReflectiveTape(data: JsonObject, transform: Pose2d): Pose2d? {
        val angle = data["angle"].asDouble.degree
        val rotation = -data["rotation"].asDouble.degree + angle + 180.degree
        val distance = data["distance"].asDouble.inch

//        println("${distance.inch}, ${angle.degree}")

        return transform + Pose2d(Translation2d(distance, angle), rotation)
    }



    val kCenterToFrontCamera = Pose2d((-1.75).inch, 0.0.inch, 0.degree)
    val kCenterToBackCamera = Pose2d((6.25).inch, 0.0.inch, 180.degree) // make sure these numbers are right
    val kRobotWidth = 30.inch
    val kRobotLength = 30.inch


}

