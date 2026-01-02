package org.firstinspires.ftc.teamcode.robot // Or your preferred package for utility classes

import android.util.Size
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.RaceAction
import com.commonlibs.units.Duration
import com.commonlibs.units.SleepAction
import com.commonlibs.units.s
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor
import org.firstinspires.ftc.vision.opencv.ImageRegion
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor


class CameraCore(
    val camera : WebcamName
) {
    enum class Balls{
        NONE,
        ARTIFACT_GREEN,
        ARTIFACT_PURPLE
    }

    var ball1 : Balls = Balls.NONE
    var ball2 : Balls = Balls.NONE
    var ball3 : Balls = Balls.NONE

    val colorSensor =
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

    val portal: VisionPortal = VisionPortal.Builder()
        .addProcessor(colorSensor)
        .setCameraResolution(Size(320, 240))
        .setCamera(camera)
        .build()

    var sensorColor = PredominantColorProcessor.Result()

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
