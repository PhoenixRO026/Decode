package org.firstinspires.ftc.teamcode.robot // Or your preferred package for utility classes

import android.util.Size
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.RaceAction
import com.commonlibs.units.Duration
import com.commonlibs.units.SleepAction
import com.commonlibs.units.Time
import com.commonlibs.units.ms
import com.commonlibs.units.s
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import org.firstinspires.ftc.teamcode.teleop.CameraConfig
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.firstinspires.ftc.vision.opencv.ImageRegion
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor
import java.util.concurrent.TimeUnit


class CameraCore(
    val cameraColor : WebcamName
) {
    @Config
    data object CameraCoreConfig {
        @JvmField
        var controller = PIDController(
            kP = 0.00001,
            kD = 0.00001,
            kI = 0.00001,
            stabilityThreshold = 50.0
        )
        @JvmField var targetRpmTolerance = 50

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

    private var exposureSet = false
    private var cameraStreamingTime = 0.ms
    private var controlsGetTime = 0.ms
    private lateinit var exposureControl: ExposureControl

    fun setExposure() {
        if (exposureSet) return

        if (portal.cameraState != VisionPortal.CameraState.STREAMING) return

        if (cameraStreamingTime <= 0.ms) {
            cameraStreamingTime = Time.now()
            return
        }

        if (Time.now() < cameraStreamingTime + 50.ms) return

        if (controlsGetTime <= 0.ms) {
            controlsGetTime = Time.now()
            exposureControl = portal.getCameraControl(ExposureControl::class.java)
            return
        }

        if (Time.now() < controlsGetTime + 50.ms) return

        exposureControl.setExposure(CameraConfig.desiredExposureMs, TimeUnit.MILLISECONDS)

        exposureSet = true
    }

    fun updateColor() {
        sensorColor = colorSensor.getAnalysis()
    }

    fun rotateToAprilTag() {
        //val pidpower = CameraCoreConfig.controller.calculate()
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
