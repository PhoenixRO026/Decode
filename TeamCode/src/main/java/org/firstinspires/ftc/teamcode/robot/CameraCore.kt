package org.firstinspires.ftc.teamcode.robot // Or your preferred package for utility classes

import android.graphics.Color
import android.util.Size
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor
import org.firstinspires.ftc.vision.opencv.ColorRange
import org.firstinspires.ftc.vision.opencv.ColorSpace
import org.firstinspires.ftc.vision.opencv.ImageRegion
import org.opencv.core.Scalar

/**
 * A dedicated class to encapsulate the setup and management of the FTC VisionPortal.
 * This makes camera initialization reusable and cleans up the OpMode.
 *
 * @param hardwareMap The HardwareMap from the OpMode, used to get the webcam.
 */
class CameraCore(
    val camera : WebcamName
) {

    // Processors for detecting green and purple artifacts
    val greenLocator: ColorBlobLocatorProcessor
    val purpleLocator: ColorBlobLocatorProcessor

    // The main VisionPortal instance
    val portal: VisionPortal

    init {
        // --- GREEN LOCATOR SETUP ---
        greenLocator = ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
            .setDrawContours(true)
            .setBoxFitColor(0)
            .setCircleFitColor(Color.rgb(0, 255, 0))
            .setBlurSize(7)
            .setDilateSize(5)
            .setErodeSize(5)
            .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
            .build()

        // --- PURPLE LOCATOR SETUP ---
        val purpleMin = Scalar(120.0, 60.0, 80.0)
        val purpleMax = Scalar(170.0, 255.0, 255.0)
        val purpleRange = ColorRange(ColorSpace.HSV, purpleMin, purpleMax)

        purpleLocator = ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(purpleRange)
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
            .setDrawContours(true)
            .setBoxFitColor(0)
            .setCircleFitColor(Color.rgb(128, 0, 128))
            .setBlurSize(7)
            .setDilateSize(5)
            .setErodeSize(5)
            .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
            .build()

        // --- VISION PORTAL SETUP ---
        portal = VisionPortal.Builder()
            .addProcessor(greenLocator)
            .addProcessor(purpleLocator)
            .setCameraResolution(Size(320, 240))
            .setCamera(camera)
            // CRITICAL: Enable the stream for the FTC Dashboard
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .build()
    }

    /**
     * A convenience method to close the portal, which should be called at the end of the OpMode.
     */
    fun close() {
        portal.close()
    }
}
