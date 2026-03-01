package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.LLResultTypes
import com.qualcomm.hardware.limelightvision.Limelight3A
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import org.firstinspires.ftc.teamcode.library.controller.LowPassFilter
import com.commonlibs.units.Duration
import kotlin.math.abs
import kotlin.math.pow

class LimeLightCore(
    val camera: Limelight3A,
    val drive: Drive
) {
    @Config
    data object LimeLightConfig {
        @JvmField
        var controller = PIDController(
            0.036,
            0.005,
            0.0007,
            newTargetReset = true,
            zeroTargetReset = true,
        )
        @JvmField var headingToleranceDeg = 1.0
        @JvmField var maxOutput = 0.6
    }

    enum class AutoCase { PPG, PGP, GPP, UNKNOWN }

    var currentCase: AutoCase = AutoCase.UNKNOWN
        private set

    var headingErrorDeg: Double = 0.0
        private set

    var previousErrorDeg : Double = 0.0
        private set

    var tagVisible: Boolean = false
        private set

    var aprilTagDistance: Double = 0.0

    private fun updateDistance() {
        val fid = camera.latestResult
            ?.fiducialResults
            ?.firstOrNull { it.fiducialId in listOf(20, 24) }
            ?: run { aprilTagDistance = Double.NaN; return }

        val pose = fid.targetPoseCameraSpace ?: run {
            aprilTagDistance = Double.NaN
            return
        }

        val x = pose.position.x
        val z = pose.position.z

        aprilTagDistance = kotlin.math.sqrt(x * x + z * z)
    }

    fun getDistance() : Double{
        updateDistance()
        return aprilTagDistance
    }
    fun setPipeline(index: Int) {
        camera.pipelineSwitch(index)
    }

    fun updateCase() {
        val id = camera.latestResult
            ?.fiducialResults
            ?.firstOrNull { it.fiducialId in listOf(21, 22, 23) }
            ?.fiducialId

        currentCase = when (id) {
            21 -> AutoCase.GPP
            22 -> AutoCase.PGP
            23 -> AutoCase.PPG
            else -> AutoCase.UNKNOWN
        }
    }

    fun getRpm(): Double {
        updateDistance()
        val rpm = -24.25548 * aprilTagDistance.pow(4.0) + 245.7718 * aprilTagDistance.pow(3.0) -
                   743.82234 * aprilTagDistance.pow(2.0) + 818.08553 * aprilTagDistance + 2723.19277
        return rpm.coerceIn(0.0, 4000.0)
    }

    fun
            updateHeadingError() {
        val fid = camera.latestResult
            ?.fiducialResults
            ?.firstOrNull()


        if (fid != null) {
            val currHeadingError = fid.targetXDegrees
            headingErrorDeg = currHeadingError
            tagVisible = true
        } else {
            headingErrorDeg = 0.0
            tagVisible = false
        }
    }

    fun computeHeadingPower(dt: Duration): Double {
        if (!tagVisible) {
//            return 0.05 * kotlin.math.sign(headingErrorDeg)
            return 0.0
        }

        var raw = LimeLightConfig.controller.calculate(0.0, headingErrorDeg, dt)
        if(abs(raw) < 0.05)
            raw = 0.0
        return raw.coerceIn(-LimeLightConfig.maxOutput, LimeLightConfig.maxOutput)
    }

    fun driveWithHeading(forward: Double = 0.0, left: Double = 0.0, dt: Duration) {
        updateHeadingError()

        val rotate = -computeHeadingPower(dt)

        drive.driveFieldCentric(forward, left, rotate)
    }
}
