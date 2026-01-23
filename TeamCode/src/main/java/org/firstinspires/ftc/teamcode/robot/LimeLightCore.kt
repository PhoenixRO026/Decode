package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.LLResultTypes
import com.qualcomm.hardware.limelightvision.Limelight3A
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import org.firstinspires.ftc.teamcode.library.controller.LowPassFilter
import com.commonlibs.units.Duration

class LimeLightCore(
    val camera: Limelight3A,
    val drive: Drive
) {
    @Config
    data object LimeLightConfig {
        @JvmField
        var controller = PIDController(
            0.015,
            0.0,
            0.001,
            0.0,
            newTargetReset = true,
            zeroTargetReset = true,
            derivativeFilter = LowPassFilter(0.0),
            stabilityThreshold = 0.0
        )
        @JvmField var headingToleranceDeg = 1.0
        @JvmField var maxOutput = 0.6
    }

    enum class AutoCase { PPG, PGP, GPP, UNKNOWN }

    var currentCase: AutoCase = AutoCase.UNKNOWN
        private set

    var headingErrorDeg: Double = 0.0
        private set


    fun setPipeline(index: Int) {
        camera.pipelineSwitch(index)
    }

    fun updateCase() {
        setPipeline(0)

        val result: LLResult? = try {
            camera.getLatestResult()
        } catch (e: Exception) {
            null
        }

        if (result == null || !result.isValid()) {
            currentCase = AutoCase.UNKNOWN
            return
        }

        val fiducials: List<LLResultTypes.FiducialResult> = result.getFiducialResults() ?: emptyList()

        val found = fiducials.firstOrNull { fr ->
            val id = try { fr.getFiducialId() } catch (e: Exception) { -1 }
            id in 21..23
        }

        currentCase = if (found != null) {
            when (found.getFiducialId()) {
                21 -> AutoCase.PPG
                22 -> AutoCase.PGP
                23 -> AutoCase.GPP
                else -> AutoCase.UNKNOWN
            }
        } else {
            AutoCase.UNKNOWN
        }
    }

    fun updateHeadingError() {
        setPipeline(1)

        val result: LLResult? = try {
            camera.getLatestResult()
        } catch (e: Exception) {
            null
        }

        if (result == null || !result.isValid()) {
            headingErrorDeg = 0.0
            return
        }

        val fiducials = result.getFiducialResults() ?: emptyList<LLResultTypes.FiducialResult>()

        // prefer a fiducial with an id in 20..24
        val chosen = fiducials.firstOrNull { fr ->
            val id = try { fr.getFiducialId() } catch (e: Exception) { -1 }
            id in 20..24
        }

        headingErrorDeg = when {
            chosen != null -> {
                // use the per-target X degrees
                try { chosen.getTargetXDegrees() } catch (e: Exception) { result.getTx() }
            }
            else -> {
                // fallback to LLResult tx (primary target)
                try { result.getTx() } catch (e: Exception) { 0.0 }
            }
        }
    }

    fun computeHeadingPower(dt: Duration): Double {
        val error = headingErrorDeg

        val raw = LimeLightConfig.controller.calculate(0.0, error, dt)

        // clamp to configured max output
        return raw.coerceIn(-LimeLightConfig.maxOutput, LimeLightConfig.maxOutput)
    }

    fun driveWithHeading(forward: Double = 0.0, left: Double = 0.0, dt: Duration) {
        updateHeadingError()

        val rotate = computeHeadingPower(dt)

        drive.driveFieldCentric(forward, left, rotate)
    }
}
