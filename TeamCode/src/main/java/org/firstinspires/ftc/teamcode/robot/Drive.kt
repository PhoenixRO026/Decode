package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.commonlibs.roadrunnerext.ex
import com.commonlibs.units.Duration
import com.commonlibs.units.Pose
import com.commonlibs.units.cm
import com.commonlibs.units.deg
import com.commonlibs.units.rotate
import com.commonlibs.units.s
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor

class  Drive(
    val mecanumDrive: MecanumDriveEx
) {
    constructor(
        hardwareMap: HardwareMap,
        pose: Pose = Pose(0.cm, 0.cm, 0.deg),
        voltageProvider: () -> Double = object : () -> Double {
            val voltageSensor = hardwareMap.getAll(VoltageSensor::class.java).first()
            override fun invoke(): Double {
                return voltageSensor.voltage
            }
        }
    ) : this(MecanumDriveEx(hardwareMap, pose, voltageProvider))
    @Config
    data object DriveConfig {
        @JvmField var slowSpeed = 0.5
    }

    private var headingOffset = 0.0
    private val heading get() = mecanumDrive.localizer.pose.heading.toDouble() - headingOffset
    private val curentSpeed get() = if (isSlowMode) DriveConfig.slowSpeed else  1.0

    var isSlowMode = false

    fun actionBuilder(beginPose: Pose, correctionTime: Duration = 0.s) = mecanumDrive.actionBuilder(beginPose.pose2d, correctionTime.asS).ex()

    fun resetFieldCentric() {
        headingOffset = mecanumDrive.localizer.pose.heading.toDouble()
    }

    fun updatePoseEstimate() {
        mecanumDrive.updatePoseEstimate()
    }

    fun driveFieldCentric(forward: Double, left: Double, rotate: Double) {
        val driveVec = PoseVelocity2d(
            Vector2d(
                forward * curentSpeed,
                left * curentSpeed
            ).rotate(-heading),
            rotate * curentSpeed
        )

        mecanumDrive.setDrivePowers(driveVec)
    }
}