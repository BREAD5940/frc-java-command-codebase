package frc.robot

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d

import com.team254.lib.physics.DCMotorTransmission
import com.team254.lib.physics.DifferentialDrive

import frc.robot.lib.motion.Util
import org.ghrobotics.lib.mathematics.units.*

public object Constants {
    /* Graciously borrowed from 5190*/
    val kRobotMass = (50f /* Robot, kg */ + 5f /* Battery, kg */ + 2f /* Bumpers, kg */).toDouble()
    val kRobotMomentOfInertia = 10.0 // kg m^2 // TODO Tune
    val kRobotAngularDrag = 12.0 // N*m / (rad/sec)

    val kWheelRadius = Util.toMeters((2f / 12f).toDouble())// meters. TODO tune
    val kTrackWidth = Util.toMeters((26f / 12f).toDouble())// meters

    // high gear: 7.5 rad/s = 15ft/sec max speed
    // acceleration: 45ft/sec^2???
    // low gear: 4.3 rad/s = 8.6ft/sec max
    // acceleration: 45ft/sec^2???
    private val kVDriveLeftLow = 0.274 * 1.0 // Volts per radians per second - Calculated emperically
    private val kADriveLeftLow = 0.032 * 1.0 // Volts per radians per second per second TODO tune
    private val kVInterceptLeftLow = 1.05 * 1.0 // Volts - tuned!

    private val kVDriveRightLow = 0.265 * 1.0 // Volts per radians per second - Calculated emperically
    private val kADriveRightLow = 0.031 * 1.0 // Volts per radians per second per second TODO tune
    private val kVInterceptRightLow = 1.02 * 1.0 // Volts - tuned!

    val kLeftTransmissionModelLowGear = DCMotorTransmission(1 / kVDriveLeftLow,
            kWheelRadius * kWheelRadius * kRobotMass / (2.0 * kADriveLeftLow),
            kVInterceptLeftLow)

    val kRightTransmissionModelLowGear = DCMotorTransmission(1 / kVDriveRightLow,
            kWheelRadius * kWheelRadius * kRobotMass / (2.0 * kADriveRightLow),
            kVInterceptRightLow)

    private val kVDriveLeftHigh = 0.143 * 1.0 // Volts per radians per second - Calculated emperically
    private val kADriveLeftHigh = 0.043 * 1.0 // Volts per radians per second per second
    private val kVInterceptLeftHigh = 1.33 * 1.0//4 * 0.4d; // Volts - tuned!

    private val kVDriveRightHigh = 0.14 * 1.0 // Volts per radians per second - Calculated emperically
    private val kADriveRightHigh = 0.043 * 1.0 // Volts per radians per second per second
    private val kVInterceptRightHigh = 1.34 * 1.0//4 * 0.4d; // Volts - tuned!

    private val kLeftTransmissionModelHighGear = DCMotorTransmission(1 / kVDriveLeftHigh,
            kWheelRadius * kWheelRadius * kRobotMass / (2.0 * kADriveLeftHigh),
            kVInterceptLeftHigh)

    private val kRightTransmissionModelHighGear = DCMotorTransmission(1 / kVDriveRightHigh,
            kWheelRadius * kWheelRadius * kRobotMass / (2.0 * kADriveRightHigh),
            kVInterceptRightHigh)

    public val kLowGearDifferentialDrive = DifferentialDrive(kRobotMass, kRobotMomentOfInertia,
            kRobotAngularDrag, kWheelRadius, kTrackWidth / 2.0, kLeftTransmissionModelLowGear, kRightTransmissionModelLowGear)

    public val kHighGearDifferentialDrive = DifferentialDrive(kRobotMass, kRobotMomentOfInertia,
            kRobotAngularDrag, kWheelRadius, kTrackWidth / 2.0, kLeftTransmissionModelHighGear, kRightTransmissionModelHighGear)

    /* Ramsete constants */
    val kDriveBeta = 2 * 1.0 // Inverse meters squared
    val kDriveZeta = 0.7 * 1.0 // Unitless dampening co-efficient

    /* Pure Pursuit constants */
    val kLat = 0.05
    val kLookaheadTime = 0.1.second
    val kMinLookaheadDistance = 2.feet

    /* Wrist stuff */
    val kWristLength = 6.inch //distance from joint to COM
    val kWristMass = 15.lb
    val kWristSpeedPerVolt = 0.21 // radians/sec/volt
    val kWristTorquePerVolt = 47.33 // Newton meters per volt, stall
    val kWristStaticFrictionVoltage = 0.0 // volts, TODO tune

    /* Elbow stuff */
    val kElbowLength = 8.inch //distance from joint to COM
    val kElbowMass = 3.lb
    val kElbowSpeedPerVolt = 0.17 // radians/sec/volt
    val kElbowTorquePerVolt = 55.0 // Newton meters per volt, stall
    val kElbowStaticFrictionVoltage = 0.0 // volts, TODO tune

    // ROBOT AND MECHANISM DIMENSIONS

    val kRobotWidth = 28.inch
    val kRobotLength = 32.inch

    val kBumperThickness = 4.5.inch
    val kIntakeProtrusionFrontExtended = 18.5.inch       // Out of frame protrusion.
    val kCenterToPointOfRotation = kRobotLength/2 - 11.inch
    val kIntakeProtrusionBackExtended = -kIntakeProtrusionFrontExtended + kCenterToPointOfRotation
    val kBadIntakeOffset = 0.inch


    // TRANSFORMATIONS
    val kFrontBumperToCenter = Pose2d(-(kRobotLength / 2.0) - kBumperThickness + kCenterToPointOfRotation, 0.meter, 0.degree)
    val kBackBumperToCenter = Pose2d((kRobotLength / 2.0) + kBumperThickness - kCenterToPointOfRotation, 0.meter, 0.degree)
    val kForwardIntakeToCenter = Pose2d(-(kRobotLength / 2.0) - kIntakeProtrusionFrontExtended, kBadIntakeOffset, 0.degree)
    val kCenterToForwardIntake = Pose2d((kRobotLength / 2.0) + kIntakeProtrusionFrontExtended, -kBadIntakeOffset, 0.degree)
    val kBackwardIntakeToCenter = Pose2d(kRobotLength / 2.0 + kIntakeProtrusionBackExtended, -kBadIntakeOffset, 0.degree)

    val kCenterToFrontCamera = Pose2d((-1.75).inch, 0.0.inch, 0.degree)
    val kCenterToBackCamera = Pose2d((-6.25).inch, 0.0.inch, 180.degree)

}
