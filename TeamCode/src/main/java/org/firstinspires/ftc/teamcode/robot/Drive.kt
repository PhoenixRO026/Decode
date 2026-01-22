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
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

class Drive(
    val mecanumDrive: MecanumDrive
) {
    @Config
    data object DriveConfig {
        @JvmField
        var slowSpeed = 0.5
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

    fun updatePoseEstimateTele(bp: BotPose?, preserveHeading: Boolean = true) {
        if (bp == null) {
            mecanumDrive.updatePoseEstimate()
            return
        }

        /**
         * Update pose during teleop using vision (or fall back to odometry).
         *
         * @param bp vision pose (null -> just run odometry)
         * @param preserveHeading when true (default) adjust headingOffset so field-centric
         *                        controls keep the same heading reference (no jerk). When false,
         *                        overwrite localizer pose directly (use this before running
         *                        an automated movement/rotation that needs exact pose).
         */
        if (preserveHeading) {
            // preserve the *driver-facing* heading used by field-centric control
            val oldHeading = mecanumDrive.localizer.getPose().heading.toDouble()
            // write vision pose into localizer
            mecanumDrive.localizer.setPose(Pose2d(bp.xMeters, bp.yMeters, bp.headingRad))
            val newHeading = mecanumDrive.localizer.getPose().heading.toDouble()
            // if localizer heading changed, update headingOffset so computed `heading` stays the same
            val delta = newHeading - oldHeading
            headingOffset += delta
        } else {
            // caller wants the localizer to match vision exactly (no offset compensation)
            mecanumDrive.localizer.setPose(Pose2d(bp.xMeters, bp.yMeters, bp.headingRad))
        }
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