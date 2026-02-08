package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.commonlibs.roadrunnerext.ex
import com.commonlibs.units.Duration
import com.commonlibs.units.Pose
import com.commonlibs.units.rotate
import com.commonlibs.units.s
import org.firstinspires.ftc.teamcode.library.controller.LowPassFilter
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

class Drive(
    val mecanumDrive: MecanumDrive
) {
    @Config
    data object DriveConfig {
        @JvmField
        var slowSpeed = 0.2
    }

    private var headingOffset = 0.0
    private val heading get() = mecanumDrive.localizer.pose.heading.toDouble() - headingOffset
    private val currentSpeed get() = if (isSlowMode) DriveConfig.slowSpeed else  1.0

    var isSlowMode = false

    fun actionBuilder(beginPose: Pose, correctionTime: Duration = 0.s) = mecanumDrive.actionBuilder(beginPose.pose2d, correctionTime.asS).ex()
    fun correctionAction(target: Pose, timeAllowed: Duration) = mecanumDrive.CorrectionAction(target, timeAllowed)
    
    fun resetFieldCentric() {
        headingOffset = mecanumDrive.localizer.pose.heading.toDouble()
    }

    fun updatePoseEstimateOdo() {
        mecanumDrive.updatePoseEstimate()
    }

    fun driveFieldCentric(forward: Double, left: Double, rotate: Double) {
        val driveVec = PoseVelocity2d(
            Vector2d(
                forward * currentSpeed,
                left * currentSpeed
            ).rotate(-heading),
            rotate * currentSpeed
        )

        mecanumDrive.setDrivePowers(driveVec)
    }
}