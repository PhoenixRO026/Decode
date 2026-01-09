package org.firstinspires.ftc.teamcode.robot

import android.util.Size
import com.acmerobotics.dashboard.config.Config
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

class AprilCamera(
    camera: WebcamName
) {
    @Config
    data object AprilConfig {
        @JvmField
        var width = 320
        @JvmField
        var height = 240
        @JvmField
        var liveView = true
        @JvmField
        var showOverlayStats = true
        @JvmField
        var drawAxes = true
        @JvmField
        var drawCubeProjection = true
        @JvmField
        var drawTagOutline = true
        @JvmField
        var drawTagId = true
    }

    private val aprilProcessor: AprilTagProcessor = AprilTagProcessor.Builder()
        .setDrawAxes(AprilConfig.drawAxes)
        .setOutputUnits(DistanceUnit.MM, AngleUnit.RADIANS)
        .setDrawCubeProjection(AprilConfig.drawCubeProjection)
        .setDrawTagOutline(AprilConfig.drawTagOutline)
        .setDrawTagID(AprilConfig.drawTagId)
        .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
        .build()

    private val portal: VisionPortal = VisionPortal.Builder()
        .addProcessor(aprilProcessor)
        .setCameraResolution(Size(AprilConfig.width, AprilConfig.height))
        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
        .setCamera(camera)
        .enableLiveView(AprilConfig.liveView)
        .setShowStatsOverlay(AprilConfig.showOverlayStats)
        .build()

    fun test() {
    }

    val cameraState: VisionPortal.CameraState by portal::cameraState
}