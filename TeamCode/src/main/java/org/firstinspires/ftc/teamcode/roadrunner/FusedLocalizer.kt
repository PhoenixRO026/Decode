package org.firstinspires.ftc.teamcode.roadrunner

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D
import org.firstinspires.ftc.teamcode.library.pedro.localization.Covariance
import org.firstinspires.ftc.teamcode.library.pedro.localization.FusionLocalizer

class FusedLocalizer(
    hardwareMap: HardwareMap,
    initialPose: Pose2d,
    val limelight3A: Limelight3A,
    val turretAngleRad: (Double) -> Unit
) {
    @Config
    data object FusionConfig {
        @JvmField var pinpointCovarianceX = 0.0
        @JvmField var pinpointCovarianceY = 0.0
        @JvmField var pinpointCovarianceH = 0.0
        @JvmField var limelightCovarianceX = 0.0
        @JvmField var limelightCovarianceY = 0.0
        @JvmField var limelightCovarianceH = 0.0
        @JvmField var bufferSize = 100
    }
    val pinpointLocalizer = PinpointLocalizer(hardwareMap, 0.0, initialPose)
    val fusedLocalizer = FusionLocalizer(
        pinpointLocalizer,
        Covariance(
            FusionConfig.limelightCovarianceX,
            FusionConfig.limelightCovarianceY,
            FusionConfig.limelightCovarianceH
        ),
        Covariance(
            FusionConfig.pinpointCovarianceX,
            FusionConfig.pinpointCovarianceY,
            FusionConfig.pinpointCovarianceH,
        ),
        Covariance(
            FusionConfig.limelightCovarianceX,
            FusionConfig.limelightCovarianceY,
            FusionConfig.limelightCovarianceH
        ),
        FusionConfig.bufferSize
    )

    fun updateLimelight() {
        val yaw = Math.toDegrees(pinpointLocalizer.pose.heading.toDouble())
        limelight3A.updateRobotOrientation(yaw)
        val result = limelight3A.latestResult ?: return
        if (result.isValid.not()) return
        val campos = flattenPose3Dto2D(result.botpose_MT2)

    }

    fun flattenPose3Dto2D(pose3D: Pose3D): Pose2d {
        val position = pose3D.position
        val unit = position.unit
        val x = DistanceUnit.INCH.fromUnit(unit, position.x)
        val y = DistanceUnit.INCH.fromUnit(unit, position.y)

        val heading = pose3D.orientation.getYaw(AngleUnit.RADIANS)

        return Pose2d(x, y, heading)
    }
}