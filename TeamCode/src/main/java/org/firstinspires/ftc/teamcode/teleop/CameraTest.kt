package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor
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

@TeleOp(name = "Testing Camera", group = "Concept")
class CameraTest : LinearOpMode() {

    override fun runOpMode() {
        val robot = Robot(hardwareMap)

        waitForStart()

        /*if (opModeIsActive()) {
            val exposureCtrl = robot.camera.portal!!.getCameraControl(ExposureControl::class.java)
            val gainCtrl = robot.camera.portal.getCameraControl(GainControl::class.java)
            val whiteBalanceCtrl =
                robot.camera.portal.getCameraControl(WhiteBalanceControl::class.java)

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
        }*/
            val exposureCtrl = robot.camera.portal!!.getCameraControl(ExposureControl::class.java)
            exposureCtrl.setExposure(CameraConfig.desiredExposureMs, TimeUnit.MILLISECONDS)


            telemetry.msTransmissionInterval = 50


            // --- MAIN LOOP ---
            // WARNING:  To view the stream preview on the Driver Station, this code runs in INIT mode.
            while (opModeIsActive() || opModeInInit()) {
                telemetry.addLine("Preview on/off: 3 dots, Camera Stream\n")

                val result: PredominantColorProcessor.Result = robot.camera.colorSensor.getAnalysis()

                // Display the Color Sensor result.
                telemetry.addData("Best Match", result.closestSwatch)
                telemetry.update()

                sleep(20)
            }
    }
}
