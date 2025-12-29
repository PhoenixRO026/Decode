package org.firstinspires.ftc.robotcontroller.external.samples

import android.util.Size
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor

@TeleOp(name = "Concept: AprilTag (Kotlin)", group = "Concept")
class ConceptAprilTagKotlin : LinearOpMode() {

    private val USE_WEBCAM = true

    private lateinit var aprilTag: AprilTagProcessor
    private lateinit var visionPortal: VisionPortal

    override fun runOpMode() {
        initAprilTag()

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream")
        telemetry.addData(">", "Touch START to start OpMode")
        telemetry.update()
        waitForStart()

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                telemetryAprilTag()
                telemetry.update()

                // Toggle streaming if needed
                if (gamepad1.dpad_down) {
                    visionPortal.stopStreaming()
                } else if (gamepad1.dpad_up) {
                    visionPortal.resumeStreaming()
                }

                sleep(20)
            }
        }

        visionPortal.close()
    }

    private fun initAprilTag() {
        // Create AprilTag processor with defaults (customize with builder methods if needed)
        aprilTag = AprilTagProcessor.Builder()
            // .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
            // .setOutputUnits(...)
            .build()

        val builder = VisionPortal.Builder()

        // Choose camera
        if (USE_WEBCAM) {
            // make sure the name matches the Robot Controller configuration
            builder.setCamera(hardwareMap.get(WebcamName::class.java, "Webcam 1"))
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK)
        }

        // Optional: request resolution and stream format (mjpeg = lower bandwidth)
        builder.setCameraResolution(Size(640, 480))
        builder.enableLiveView(true)
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG)

        // If you ever remove processors but still want the live feed, prevent auto-stop:
        builder.setAutoStopLiveView(false)

        // Add AprilTag processor (omit this line if you only want plain streaming)
        builder.addProcessor(aprilTag)

        // Build
        visionPortal = builder.build()
    }

    private fun telemetryAprilTag() {
        val currentDetections: List<AprilTagDetection> = aprilTag.getDetections()
        telemetry.addData("# AprilTags Detected", currentDetections.size)

        for (detection in currentDetections) {
            val md = detection.metadata
            if (md != null) {
                telemetry.addLine("\n==== (ID ${detection.id}) ${md.name}")
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z))
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw))
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation))
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id))
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y))
            }
        }

        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.")
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)")
        telemetry.addLine("RBE = Range, Bearing & Elevation")
    }
}
