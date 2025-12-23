package org.firstinspires.ftc.robotcontroller.external.samples

import android.graphics.Color
import android.util.Size
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.commonlibs.units.SleepAction
import com.commonlibs.units.s
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor
import org.firstinspires.ftc.vision.opencv.ColorRange
import org.firstinspires.ftc.vision.opencv.ColorSpace
import org.firstinspires.ftc.vision.opencv.ImageRegion
import org.opencv.core.Scalar
import java.util.concurrent.TimeUnit

@Config
data object CameraTuning{
    // --- Enforce manual exposure & gain to keep colors stable ---
    // Tune these values for your lighting. If the device doesn't support the control the call is no-op.
    @JvmField
    var desiredExposureMs = 10L   // 8-12 is a good starting point for normal lighting
    @JvmField
    var desiredGain = 14          // low gain reduces noise
    @JvmField
    var V = 60.0
    @JvmField
    var whiteCv: Long = 1
}

@TeleOp(name = "Detect: Artifact Green/Purple (Robust)", group = "Concept")
class DetectArtifactColorRobust : LinearOpMode() {

    override fun runOpMode() {
        // GREEN: keep predefined (usually safe)
        val greenLocator = ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)
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

        // PURPLE: use a custom HSV range that EXCLUDES very low V (brightness) values (i.e. black)
        // Note: OpenCV HSV H range is 0..180, S and V are 0..255
        // The H/S window here targets purples/magentas while enforcing V >= 40 (raised to taste)
        val purpleMin = Scalar(120.0, 60.0, CameraTuning.V)   // H=120, S=60, V=40 (V=40 excludes near-black)
        val purpleMax = Scalar(170.0, 255.0, 255.0) // H=170, S max, V max

        val purpleRange = ColorRange(ColorSpace.HSV, purpleMin, purpleMax)

        val purpleLocator = ColorBlobLocatorProcessor.Builder()
            .setTargetColorRange(purpleRange)
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.ALL_FLATTENED_HIERARCHY)
            .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
            .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
            .setDrawContours(true)
            .setBoxFitColor(0)
            .setCircleFitColor(Color.rgb(128, 0, 128))
            .setBlurSize(7)
            // reduce the extreme dilation/erosion — aggressive dilation can merge noise into blobs
            .setDilateSize(5)
            .setErodeSize(5)
            .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
            .build()

        // Build the vision portal with both processors
        val portal = VisionPortal.Builder()
            .addProcessor(greenLocator)
            .addProcessor(purpleLocator)
            .setCameraResolution(Size(320, 240))
            .setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
            .build()

        waitForStart()

        // --- SETUP: Get controls and set modes to MANUAL ONCE ---
        var exposureCtrl = portal.getCameraControl(ExposureControl::class.java)
        var gainCtrl = portal.getCameraControl(GainControl::class.java)
        var whiteBalanceCtrl = portal.getCameraControl(WhiteBalanceControl::class.java)


        val exposureAction = SequentialAction(
            InstantAction { portal.cameraState != VisionPortal.CameraState.STREAMING },
            SleepAction(0.5.s),
            InstantAction {
                exposureCtrl = portal.getCameraControl(ExposureControl::class.java)
                gainCtrl = portal.getCameraControl(GainControl::class.java)
                exposureCtrl.setMode(ExposureControl.Mode.Manual)
            },
            SleepAction(0.5.s),
            InstantAction {
                exposureCtrl.setExposure(CameraTuning.whiteCv, TimeUnit.MILLISECONDS)
            },
            SleepAction(0.5.s),
            InstantAction {
                gainCtrl.setGain(CameraTuning.desiredGain)
            }
        )

        telemetry.msTransmissionInterval = 50 // Faster telemetry updates

        // Filters: tightened to avoid tiny dark blobs turning into false positives
        val minArea = 500.0    // raise from 50 to 150
        val maxArea = 20000.0
        val minCircularity = 0.6
        val minDensity = 0.5   // require reasonably "solid" blobs

        while (opModeIsActive()) {




            // --- TELEMETRY: Display the ACTUAL values the camera is using ---
            telemetry.addLine("--- Camera Controls ---")
            telemetry.addData("Exposure Actual", "%d ms", exposureCtrl.getExposure(TimeUnit.MILLISECONDS))
            telemetry.addData("Gain Actual", gainCtrl.gain)
            telemetry.addData("WB Mode", whiteBalanceCtrl.mode)
            telemetry.addData("WB Temp Actual", whiteBalanceCtrl.whiteBalanceTemperature) // Show actual temp
            telemetry.addLine()
            telemetry.addData("Exposure Desired", CameraTuning.desiredExposureMs)
            telemetry.addData("Gain Desired", CameraTuning.desiredGain)
            telemetry.addData("WB Temp Desired", CameraTuning.whiteCv) // Show desired temp
            telemetry.addLine(" ")

            val greenBlobs = greenLocator.blobs
            val purpleBlobs = purpleLocator.blobs

            // Filter both lists
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                minArea, maxArea, greenBlobs
            )
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                minCircularity, 1.0, greenBlobs
            )
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_DENSITY,
                minDensity, 1.0, greenBlobs
            )

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                minArea, maxArea, purpleBlobs
            )
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                minCircularity, 1.0, purpleBlobs
            )
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_DENSITY,
                minDensity, 1.0, purpleBlobs
            )

            // Helper: largest by circle radius
            fun strongestBlob(blobs: List<ColorBlobLocatorProcessor.Blob>): ColorBlobLocatorProcessor.Blob? {
                if (blobs.isEmpty()) return null
                return blobs.maxByOrNull { it.circle.radius }
            }

            val gBlob = strongestBlob(greenBlobs)
            val pBlob = strongestBlob(purpleBlobs)

            val seenColor = when {
                gBlob == null && pBlob == null -> "NONE"
                gBlob != null && pBlob == null -> "GREEN"
                pBlob != null && gBlob == null -> "PURPLE"
                else -> if (gBlob!!.circle.radius >= pBlob!!.circle.radius) "GREEN" else "PURPLE"
            }

            telemetry.addData("Detected", seenColor)
            telemetry.addLine("Green blobs: ${greenBlobs.size}")
            gBlob?.let {
                telemetry.addLine(String.format("  G - r=%3d, circ=%4.3f, dens=%4.3f, center=(%3d,%3d)",
                    it.circle.radius.toInt(), it.circularity, it.density, it.circle.x.toInt(), it.circle.y.toInt()))
            }

            telemetry.addLine("Purple blobs: ${purpleBlobs.size}")
            pBlob?.let {
                telemetry.addLine(String.format("  P - r=%3d, circ=%4.3f, dens=%4.3f, center=(%3d,%3d)",
                    it.circle.radius.toInt(), it.circularity, it.density, it.circle.x.toInt(), it.circle.y.toInt()))
            }

            telemetry.update()
            sleep(100)
        }
    }
}
