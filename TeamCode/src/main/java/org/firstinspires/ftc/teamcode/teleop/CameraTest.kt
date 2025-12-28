package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor
import java.util.concurrent.TimeUnit

@Config
data object CameraConfig{
    @JvmField
    var desiredExposureMs : Long = 20
    @JvmField
    var desiredGain = 25
    @JvmField
    var V = 60.0
    @JvmField
    var whiteValue = 5000 // Use Int or Long, but be consistent. WhiteBalance takes Int.
}

@TeleOp(name = "Detect: Artifact Green/Purple (Robust)", group = "Concept")
class CameraTest : LinearOpMode() {

    override fun runOpMode() {
        // --- INITIALIZE THE CAMERA SYSTEM USING YOUR NEW CLASS ---
        val robot = Robot(hardwareMap)

        waitForStart()

        if (!opModeIsActive()) {
            robot.camera.close()
            return
        }

        // Wait for the camera to be streaming before trying to use controls
        while (opModeIsActive() && robot.camera.portal.cameraState != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Waiting for camera to start streaming...")
            telemetry.update()
            sleep(20)
        }

        // --- SETUP CONTROLS (only if opMode is still active after waiting) ---
        if (opModeIsActive()) {
            val exposureCtrl = robot.camera.portal.getCameraControl(ExposureControl::class.java)
            val gainCtrl = robot.camera.portal.getCameraControl(GainControl::class.java)
            val whiteBalanceCtrl = robot.camera.portal.getCameraControl(WhiteBalanceControl::class.java)

            // Set camera controls to manual
            try {
                if (exposureCtrl.isModeSupported(ExposureControl.Mode.Manual)) {
                    exposureCtrl.mode = ExposureControl.Mode.Manual
                }
                // WhiteBalanceControl does not have isModeSupported, so we just try it
                whiteBalanceCtrl.mode = WhiteBalanceControl.Mode.MANUAL
            } catch (e: Exception) {
                telemetry.addLine("Error setting manual camera modes. Camera may be stuck in AUTO.")
                telemetry.addData("Error", e.message)
            }

            telemetry.msTransmissionInterval = 50

            // Filters
            val minArea = 500.0
            val maxArea = 20000.0
            val minCircularity = 0.6
            val minDensity = 0.5

            // --- MAIN LOOP ---
            while (opModeIsActive()) {
                // Apply dashboard tuning values on every loop
                try {
                    exposureCtrl.setExposure(CameraConfig.desiredExposureMs, TimeUnit.MILLISECONDS)
                    gainCtrl.gain = CameraConfig.desiredGain
                    whiteBalanceCtrl.setWhiteBalanceTemperature(CameraConfig.whiteValue)
                } catch (e: Exception) {
                    // Ignore errors here to prevent spamming the log
                }

                // --- TELEMETRY ---
                telemetry.addLine("--- Camera Controls ---")
                telemetry.addData("Exposure Actual", "%d ms", exposureCtrl.getExposure(TimeUnit.MILLISECONDS))
                telemetry.addData("Gain Actual", gainCtrl.gain)
                telemetry.addData("WB Temp Actual", whiteBalanceCtrl.whiteBalanceTemperature)
                telemetry.addLine()
                telemetry.addData("WB Temp Desired", CameraConfig.whiteValue)
                telemetry.addLine(" ")

                val greenBlobs = robot.camera.greenLocator.blobs
                val purpleBlobs = robot.camera.purpleLocator.blobs

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

        // --- CLEANUP ---
        robot.camera.close()
    }
}
