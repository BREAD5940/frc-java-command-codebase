package frc.robot

import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab

object Network {

    val AutoTab by lazy {Shuffleboard.getTab("Auto")}

    val DriveTrainTab by lazy {Shuffleboard.getTab("DriveTrain")}

    val SuperStructureTab by lazy {Shuffleboard.getTab("SuperStructure")}

    val IntakeTab: ShuffleboardTab by lazy {Shuffleboard.getTab("Intake")}

    val VisionTab: ShuffleboardTab by lazy {Shuffleboard.getTab("Vision")}

    private val visionLayout = VisionTab.getLayout("Vision", BuiltInLayouts.kGrid)
            .withSize(3, 3)
//            .withPosition(0, 2)

    val visionDriveAngle: NetworkTableEntry = visionLayout.add("Vision Drive Angle", 0.0).entry
    val visionDriveActive: NetworkTableEntry = visionLayout.add("Vision Drive Active", false).entry

}