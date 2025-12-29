package org.firstinspires.ftc.teamcode.robot_old

import android.util.Size
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.opencv.ImageRegion
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor

/**
 * A dedicated class to encapsulate the setup and management of the FTC VisionPortal.
 * This makes camera initialization reusable and cleans up the OpMode.
 *
 * @param hardwareMap The HardwareMap from the OpMode, used to get the webcam.
 */
class CameraCore(
    val camera : WebcamName
) {
        val colorSensor =
            PredominantColorProcessor.Builder()
                //.setRoi(ImageRegion.asUnityCenterCoordinates(-0.25, 0.25, 0.25, -0.25))
                .setRoi(ImageRegion.entireFrame())
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
}