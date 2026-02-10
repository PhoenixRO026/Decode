package org.firstinspires.ftc.teamcode.robot // Or your preferred package for utility classes

import android.util.Size
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.RaceAction
import com.commonlibs.units.Duration
import com.commonlibs.units.SleepAction
import com.commonlibs.units.s
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.opencv.ImageRegion
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor


class CameraCore(
    val cameraColor : WebcamName
) {
    @Config
    data object CameraCoreConfig {

        @JvmField var colorWidth = 160
        @JvmField var colorHeight = 120
    }

    var sensorColor = PredominantColorProcessor.Result()

    val colorSensor =
        PredominantColorProcessor.Builder()
            .setRoi(ImageRegion.asUnityCenterCoordinates(0.25, 0.25, 1.0, -0.25))
            .setSwatches(
                PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                PredominantColorProcessor.Swatch.BLACK,
            )
            .build()

    val portal: VisionPortal? = VisionPortal.Builder()
        .addProcessor(colorSensor)
        .setCameraResolution(Size(CameraCoreConfig.colorWidth, CameraCoreConfig.colorHeight))
        .setCamera(cameraColor)
        .build()


    fun updateColor() {
        sensorColor = colorSensor.getAnalysis()
    }

    private fun waitForColor(color: PredominantColorProcessor.Swatch, maxTime : Duration = 10.s) = RaceAction(
        {
            updateColor()
            it.addLine("waiting for color")
            sensorColor.closestSwatch != color
        },
        SleepAction(maxTime)
    )

    fun waitForColors(maxTime : Duration = 10.s) = RaceAction(
        waitForColor(PredominantColorProcessor.Swatch.ARTIFACT_GREEN, maxTime),
        waitForColor(PredominantColorProcessor.Swatch.ARTIFACT_PURPLE, maxTime)
    )

}
