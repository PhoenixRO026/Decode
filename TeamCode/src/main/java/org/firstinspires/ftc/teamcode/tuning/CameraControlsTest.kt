package org.firstinspires.ftc.teamcode.tuning

import android.util.Size
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.commonlibs.units.Time
import com.commonlibs.units.ms
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl
import org.firstinspires.ftc.vision.VisionPortal
import java.util.concurrent.TimeUnit

@TeleOp(group = "tuning")
class CameraControlsTest : LinearOpMode() {
    @Config("Camera controls test config")
    companion object {
        @JvmField
        var cooldownMs = 50
        @JvmField var width = 1280
        @JvmField var height = 720
        @JvmField var liveView = true
        @JvmField
        var exposureMode = ExposureControl.Mode.Unknown
        @JvmField
        var exposureIsAutoModeSupported = false
        @JvmField
        var exposureIsContinuousAutoModeSupported = false
        @JvmField
        var exposureIsManualModeSupported = false
        @JvmField
        var exposureIsShutterPriorityModeSupported = false
        @JvmField
        var exposureIsAperturePriorityModeSupported = false
        @JvmField
        var minExposureMs = 0L
        @JvmField
        var maxExposureMs = 0L
        @JvmField
        var currentExposureMs = 0L
        @JvmField
        var isExposureSupported = false
        @JvmField
        var exposureAePriority = false
        @JvmField
        var focusMode = FocusControl.Mode.Unknown
        @JvmField
        var focusIsAutoModeSupported = false
        @JvmField
        var focusIsContinuousAutoModeSupported = false
        @JvmField
        var focusIsMacroModeSupported = false
        @JvmField
        var focusIsInfinityModeSupported = false
        @JvmField
        var focusIsFixedModeSupported = false
        @JvmField
        var minFocusLength = -1.0
        @JvmField
        var maxFocusLength = -1.0
        @JvmField
        var currentFocusLength = -1.0
        @JvmField
        var isFocusLengthSupported = false
        @JvmField
        var minGain = 0
        @JvmField
        var maxGain = 0
        @JvmField
        var currentGain = 0
        @JvmField
        var currentPanTilt = PtzControl.PanTiltHolder()
        @JvmField
        var minPanTilt = PtzControl.PanTiltHolder()
        @JvmField
        var maxPanTilt = PtzControl.PanTiltHolder()
        @JvmField
        var currentZoom = 0
        @JvmField
        var minZoom = 0
        @JvmField
        var maxZoom = 0
        @JvmField
        var whiteBalanceMode = WhiteBalanceControl.Mode.UNKNOWN
        @JvmField
        var minWhiteBalanceTemperature = 0
        @JvmField
        var maxWhiteBalanceTemperature = 0
        @JvmField
        var currentWhiteBalanceTemperature = 0
    }

    private var cameraInitDone = false
    private var controlsCreated = false
    private var specsDone = false
    private var lastCommandTime = Time.now()
    private var specsProgress = 0
    private lateinit var exposureControl: ExposureControl
    private lateinit var focusControl: FocusControl
    private lateinit var gainControl: GainControl
    private lateinit var ptzControl: PtzControl
    private lateinit var whiteBalanceControl: WhiteBalanceControl
    private lateinit var visionPortal: VisionPortal

    override fun runOpMode() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        val webcamName = hardwareMap.get(WebcamName::class.java, "Webcam 1")
        visionPortal = VisionPortal.Builder()
            .setCamera(webcamName)
            .setCameraResolution(Size(width, height))
            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
            .enableLiveView(liveView)
            .build()

        while (opModeInInit()) {
            if (visionPortal.cameraState != VisionPortal.CameraState.STREAMING) continue
            if (!cameraInitDone) cameraInit()

            addTelemetry()
        }

        while (opModeIsActive()) {
            if (visionPortal.cameraState != VisionPortal.CameraState.STREAMING) continue
            if (!cameraInitDone) cameraInit()

            addTelemetry()
        }
    }

    fun cameraInit() {
        if (!controlsCreated) {
            createControls()
            resetCooldown()
        }
        if (!specsDone) {
            getSpecs()
            return
        }
        cameraInitDone = true
    }

    fun onCooldown(): Boolean = Time.now() - lastCommandTime < cooldownMs.ms
    fun resetCooldown() {
        lastCommandTime = Time.now()
    }

    fun createControls() {
        controlsCreated = true
        exposureControl = visionPortal.getCameraControl(ExposureControl::class.java)
        focusControl = visionPortal.getCameraControl(FocusControl::class.java)
        gainControl = visionPortal.getCameraControl(GainControl::class.java)
        ptzControl = visionPortal.getCameraControl(PtzControl::class.java)
        whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl::class.java)
    }

    fun getSpecs() {
        if (onCooldown()) return
        resetCooldown()
        when (specsProgress) {
            0 -> exposureMode = exposureControl.mode
            1 -> exposureIsAutoModeSupported =
                exposureControl.isModeSupported(ExposureControl.Mode.Auto)

            2 -> exposureIsContinuousAutoModeSupported =
                exposureControl.isModeSupported(ExposureControl.Mode.ContinuousAuto)

            3 -> exposureIsManualModeSupported =
                exposureControl.isModeSupported(ExposureControl.Mode.Manual)

            4 -> exposureIsShutterPriorityModeSupported =
                exposureControl.isModeSupported(ExposureControl.Mode.ShutterPriority)

            5 -> exposureIsAperturePriorityModeSupported =
                exposureControl.isModeSupported(ExposureControl.Mode.AperturePriority)

            6 -> minExposureMs = exposureControl.getMinExposure(TimeUnit.MILLISECONDS)
            7 -> maxExposureMs = exposureControl.getMaxExposure(TimeUnit.MILLISECONDS)
            8 -> currentExposureMs = exposureControl.getExposure(TimeUnit.MILLISECONDS)
            9 -> isExposureSupported = exposureControl.isExposureSupported
            10 -> exposureAePriority = exposureControl.aePriority
            11 -> focusMode = focusControl.mode
            12 -> focusIsAutoModeSupported = focusControl.isModeSupported(FocusControl.Mode.Auto)
            13 -> focusIsContinuousAutoModeSupported =
                focusControl.isModeSupported(FocusControl.Mode.ContinuousAuto)

            14 -> focusIsMacroModeSupported = focusControl.isModeSupported(FocusControl.Mode.Macro)
            15 -> focusIsInfinityModeSupported =
                focusControl.isModeSupported(FocusControl.Mode.Infinity)

            16 -> focusIsFixedModeSupported = focusControl.isModeSupported(FocusControl.Mode.Fixed)
            17 -> minFocusLength = focusControl.minFocusLength
            18 -> maxFocusLength = focusControl.maxFocusLength
            19 -> currentFocusLength = focusControl.focusLength
            20 -> isFocusLengthSupported = focusControl.isFocusLengthSupported
            21 -> minGain = gainControl.minGain
            22 -> maxGain = gainControl.maxGain
            23 -> currentGain = gainControl.gain
            24 -> currentPanTilt = ptzControl.panTilt
            25 -> minPanTilt = ptzControl.minPanTilt
            26 -> maxPanTilt = ptzControl.maxPanTilt
            27 -> currentZoom = ptzControl.zoom
            28 -> minZoom = ptzControl.minZoom
            29 -> maxZoom = ptzControl.maxZoom
            30 -> whiteBalanceMode = whiteBalanceControl.mode
            31 -> minWhiteBalanceTemperature = whiteBalanceControl.minWhiteBalanceTemperature
            32 -> maxWhiteBalanceTemperature = whiteBalanceControl.maxWhiteBalanceTemperature
            33 -> currentWhiteBalanceTemperature = whiteBalanceControl.whiteBalanceTemperature
            34 -> specsDone = true
        }
        specsProgress++
    }

    fun addTelemetry() {
        telemetry.addData("fps", visionPortal.fps)

        val dataList = listOf(
            Pair("exposureMode", exposureMode),
            Pair("exposureIsAutoModeSupported", exposureIsAutoModeSupported),
            Pair("exposureIsContinuousAutoModeSupported", exposureIsContinuousAutoModeSupported),
            Pair("exposureIsManualModeSupported", exposureIsManualModeSupported),
            Pair("exposureIsShutterPriorityModeSupported", exposureIsShutterPriorityModeSupported),
            Pair(
                "exposureIsAperturePriorityModeSupported",
                exposureIsAperturePriorityModeSupported
            ),
            Pair("minExposureMs", minExposureMs),
            Pair("maxExposureMs", maxExposureMs),
            Pair("currentExposureMs", currentExposureMs),
            Pair("isExposureSupported", isExposureSupported),
            Pair("exposureAePriority", exposureAePriority),
            Pair("focusMode", focusMode),
            Pair("focusIsAutoModeSupported", focusIsAutoModeSupported),
            Pair("focusIsContinuousAutoModeSupported", focusIsContinuousAutoModeSupported),
            Pair("focusIsMacroModeSupported", focusIsMacroModeSupported),
            Pair("focusIsInfinityModeSupported", focusIsInfinityModeSupported),
            Pair("focusIsFixedModeSupported", focusIsFixedModeSupported),
            Pair("minFocusLength", minFocusLength),
            Pair("maxFocusLength", maxFocusLength),
            Pair("currentFocusLength", currentFocusLength),
            Pair("isFocusLengthSupported", isFocusLengthSupported),
            Pair("minGain", minGain),
            Pair("maxGain", maxGain),
            Pair("currentGain", currentGain),
            Pair("currentPanTilt", currentPanTilt),
            Pair("minPanTilt", minPanTilt),
            Pair("maxPanTilt", maxPanTilt),
            Pair("currentZoom", currentZoom),
            Pair("minZoom", minZoom),
            Pair("maxZoom", maxZoom),
            Pair("whiteBalanceMode", whiteBalanceMode),
            Pair("minWhiteBalanceTemperature", minWhiteBalanceTemperature),
            Pair("maxWhiteBalanceTemperature", maxWhiteBalanceTemperature),
            Pair("currentWhiteBalanceTemperature", currentWhiteBalanceTemperature),
            Pair("specsDone", specsDone)
        )

        dataList.forEach {
            telemetry.addData(it.first, it.second)
        }
        telemetry.update()
    }
}