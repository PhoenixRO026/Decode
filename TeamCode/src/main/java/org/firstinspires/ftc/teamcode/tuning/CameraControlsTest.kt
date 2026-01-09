package org.firstinspires.ftc.teamcode.tuning

import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.CameraControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl
import org.firstinspires.ftc.vision.VisionPortal

@TeleOp
class CameraControlsTest: LinearOpMode() {
    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val webcamName = hardwareMap.get(WebcamName::class.java, "Webcam 1")
        val visionPortal = VisionPortal.Builder()
            .setCamera(webcamName)
            .setCameraResolution(Size(1280, 720))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .build()

        var controlsCreated = false
        lateinit var exposureControl: ExposureControl
        lateinit var focusControl: FocusControl
        lateinit var gainControl: GainControl
        lateinit var ptzControl: PtzControl
        lateinit var whiteBalanceControl: WhiteBalanceControl

        waitForStart()

        while (opModeIsActive()) {
            if (visionPortal.cameraState != VisionPortal.CameraState.STREAMING) continue
            if (!controlsCreated) {
                controlsCreated = true
                exposureControl = visionPortal.getCameraControl(ExposureControl::class.java)
                focusControl = visionPortal.getCameraControl(FocusControl::class.java)
                gainControl = visionPortal.getCameraControl(GainControl::class.java)
                ptzControl = visionPortal.getCameraControl(PtzControl::class.java)
                whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl::class.java)
            }


        }
    }
}