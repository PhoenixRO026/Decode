package org.firstinspires.ftc.teamcode.robot

import android.util.Size
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.RaceAction
import com.commonlibs.units.Duration
import com.commonlibs.units.SleepAction
import com.commonlibs.units.s
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.opencv.ImageRegion
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor


class ColorCamera(
    camera : WebcamName
) {
    @Config
    data object ColorConfig {
        @JvmField
        var width = 320
        @JvmField
        var height = 240
        @JvmField
        var liveView = true
        @JvmField
        var showOverlayStats = true
    }

    private val colorProcessor: PredominantColorProcessor =
        PredominantColorProcessor.Builder()
            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.5, 0.25, 1.0, -0.5))
            .setSwatches(
                PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                PredominantColorProcessor.Swatch.ORANGE,
                PredominantColorProcessor.Swatch.BLACK,
                PredominantColorProcessor.Swatch.WHITE
            )
            .build()

    private val portal: VisionPortal = VisionPortal.Builder()
        .addProcessor(colorProcessor)
        .setCameraResolution(Size(ColorConfig.width, ColorConfig.height))
        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
        .setCamera(camera)
        .enableLiveView(ColorConfig.liveView)
        .setShowStatsOverlay(ColorConfig.showOverlayStats)
        .build()

    val cameraState: VisionPortal.CameraState by portal::cameraState

    val detectedColor get() = when (colorProcessor.analysis.closestSwatch) {
        PredominantColorProcessor.Swatch.ARTIFACT_GREEN -> BallStorage.Storage.GREEN
        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE -> BallStorage.Storage.PURPLE
        else -> BallStorage.Storage.NONE
    }

    private fun waitForColor(color: BallStorage.Storage, maxTime : Duration = 10.s) = RaceAction(
        {
            it.addLine("waiting for color")
            detectedColor != color
        },
        SleepAction(maxTime)
    )

    fun waitForColors(maxTime : Duration = 10.s) = RaceAction(
            waitForColor(BallStorage.Storage.GREEN, maxTime),
            waitForColor(BallStorage.Storage.PURPLE, maxTime)
        )

}
