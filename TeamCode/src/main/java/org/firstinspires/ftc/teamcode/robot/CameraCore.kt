package org.firstinspires.ftc.teamcode.robot // Or your preferred package for utility classes

import android.util.Size
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.RaceAction
import com.commonlibs.units.Duration
import com.commonlibs.units.SleepAction
import com.commonlibs.units.s
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.vision.opencv.ImageRegion
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor


class CameraCore(
    val cameraColor : WebcamName,
    val cameraAprilTag : WebcamName
) {
    @Config
    data object CameraCoreConfig {
        @JvmField var colorWidth = 432
        @JvmField var colorHeight = 240
        @JvmField var aprilTagWidth = 1280
        @JvmField var aprilTagHeight = 720
        @JvmField var liveView = false
        @JvmField var decimation = 2.0f
    }
    enum class Balls{
        NONE,
        ARTIFACT_GREEN,
        ARTIFACT_PURPLE
    }

    enum class Cases{
        PPG,
        PGP,
        GPP
    }

    var ball1 : Balls = Balls.NONE
    var ball2 : Balls = Balls.NONE
    var ball3 : Balls = Balls.NONE

    var sensorColor = PredominantColorProcessor.Result()

    val viewIds: IntArray =
        VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL)

    // We extract the two view IDs from the array to make our lives a little easier later.
    // NB: the array is 2 long because we asked for 2 portals up above.
    val portal1ViewId: Int = viewIds[0]
    val portal2ViewId: Int = viewIds[1]

    val aprilTag = AprilTagProcessor.Builder()
        .setSuppressCalibrationWarnings(true)
        .setDrawTagID(CameraCoreConfig.liveView)
        .setDrawAxes(CameraCoreConfig.liveView)
        .setDrawCubeProjection(CameraCoreConfig.liveView)
        .setDrawTagOutline(CameraCoreConfig.liveView)
        .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
        .build().apply {
            setDecimation(CameraCoreConfig.decimation)
        }

    val visionPortal = VisionPortal.Builder()
        .setCamera(cameraAprilTag)
        .setCameraResolution(Size(CameraCoreConfig.aprilTagWidth, CameraCoreConfig.aprilTagHeight))
        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
        .addProcessor(aprilTag)
        .setLiveViewContainerId(portal2ViewId)
        //.enableLiveView(CameraCoreConfig.liveView)
        .build()

    private var id = 0

    fun detectAprilTagCase() : Int {
        val currentDetections: List<AprilTagDetection> = aprilTag.detections
        for (detection in currentDetections) {
            if (detection.metadata != null) {
                if (detection.id > 20 && detection.id < 24) {
                    id = detection.id
                }
            }
        }
        return id
    }

    val detectedCase = detectAprilTagCase()

    fun stopStream() {
        visionPortal.stopStreaming()
    }

    fun resumeStream() {
        visionPortal.resumeStreaming()
    }

    val colorSensor =
        PredominantColorProcessor.Builder()
            .setRoi(ImageRegion.asUnityCenterCoordinates(0.0, 0.25, 1.0, -0.8))
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
        .setCameraResolution(Size(CameraCoreConfig.colorWidth, CameraCoreConfig.colorHeight))
        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
        .setCamera(cameraColor)
        .setLiveViewContainerId(portal1ViewId)
        //.enableLiveView(CameraCoreConfig.liveView)
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
