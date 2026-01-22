package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.LLResultTypes
import kotlin.math.hypot

data class BotPose(
    val xMeters: Double,
    val yMeters: Double,
    val headingRad: Double
)

enum class AutoCase {
    NONE,
    CASE_21,
    CASE_22,
    CASE_23
}

class LimelightCore(
    private val limelight: Limelight3A
) {
    /** Call once during init */
    fun init(pipeline: Int = 0, pollRateHz: Int = 100) {
        limelight.pipelineSwitch(pipeline)
        limelight.setPollRateHz(pollRateHz)
        limelight.start()
    }

    /** Expose latest raw LLResult (null-safe if invalid) */
    fun latestResult(): LLResult? {
        val r = limelight.latestResult
        return if (r != null && r.isValid) r else null
    }

    /** Get raw fiducials list (may be empty) */
    fun getFiducials(): List<LLResultTypes.FiducialResult> {
        val r = latestResult() ?: return emptyList()
        // fiducialResults is a List<LLResultTypes.FiducialResult> in the Java driver
        return r.fiducialResults ?: emptyList()
    }

    /** Detect which AprilTag case we see (21 / 22 / 23) */
    fun detectAutoCase(): AutoCase {
        val r = latestResult() ?: return AutoCase.NONE
        val fiducials = r.fiducialResults ?: return AutoCase.NONE
        for (tag in fiducials) {
            when (tag.fiducialId) {
                21 -> return AutoCase.CASE_21
                22 -> return AutoCase.CASE_22
                23 -> return AutoCase.CASE_23
            }
        }
        return AutoCase.NONE
    }

    /** Returns robot pose from Limelight 3D solve (meters + radians) */
    fun getBotPose(): BotPose? {
        val r = latestResult() ?: return null
        val p = r.botpose ?: return null
        return BotPose(
            xMeters = p.position.x,
            yMeters = p.position.y,
            headingRad = p.orientation.yaw
        )
    }

    /** Helper to know if limelight currently has a 3D pose */
    fun hasBotPose(): Boolean {
        return getBotPose() != null
    }

    fun stop() {
        limelight.stop()
    }
}
