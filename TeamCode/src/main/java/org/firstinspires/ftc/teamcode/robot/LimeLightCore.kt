package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.LLResultTypes
import com.qualcomm.hardware.limelightvision.Limelight3A
import kotlin.math.abs

class LimeLightCore(
    val camera: Limelight3A
) {
    @Config
    data object LimeLightConfig {
        @JvmField var kP = 0.015
        @JvmField var kI = 0.0
        @JvmField var kD = 0.001
        @JvmField var headingToleranceDeg = 1.0
        @JvmField var maxOutput = 0.6
    }

    enum class AutoCase { PPG, PGP, GPP, UNKNOWN }

    var currentCase: AutoCase = AutoCase.UNKNOWN
        private set

    var headingErrorDeg: Double = 0.0
        private set

    private var integral = 0.0
    private var lastError = 0.0


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
    fun updateHeading() {
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


    fun headingPower(): Double {
        val error = headingErrorDeg

        if (abs(error) <= LimeLightConfig.headingToleranceDeg) {
            integral = 0.0
            lastError = error
            return 0.0
        }

        integral += error
        val derivative = error - lastError
        lastError = error

        var output = error * LimeLightConfig.kP + integral * LimeLightConfig.kI + derivative * LimeLightConfig.kD
        output = output.coerceIn(-LimeLightConfig.maxOutput, LimeLightConfig.maxOutput)
        return output
    }

    fun resetPID() {
        integral = 0.0
        lastError = 0.0
    }
}
