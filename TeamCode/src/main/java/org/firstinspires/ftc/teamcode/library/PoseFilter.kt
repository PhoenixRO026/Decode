package org.firstinspires.ftc.teamcode.library

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.commonlibs.units.rotate
import java.util.NavigableMap
import java.util.TreeMap
import kotlin.math.abs
import kotlin.math.absoluteValue

class PoseFilter {
    @Config
    data object PoseFilterConfig {
        // q: process noise covariance how fast odometry drifts
        @JvmField var Q_POS = 0.002
        @JvmField var Q_HEAD = 0.001

        // r: measurement noisy covariance
        @JvmField var R_POS = 0.5   //inch^2 variance
        @JvmField var R_HEAD = 0.05 //rad^2 variance

        // outliers
        @JvmField var GATE_DISTANCE = 6.0   // inch
        @JvmField var GATE_HEADING = Math.toRadians(15.0)
        @JvmField var OUTLIER_COUNT = 10

        @JvmField var HISTORY_RETENTION_MS = 1000L
    }

    private val odomHistory: NavigableMap<Long, Pose2d> = TreeMap()
    private var offset = Pose2d(0.0, 0.0, 0.0)

    private var covX = 0.0
    private var covY = 0.0
    private var covH = 0.0

    private var suspiciousReadingCounter = 0
    private var isVisionInitialized = false
    private var pastVisionTimestamp = 0.0

    fun getPose(rawPose: Pose2d): Pose2d {
        return offset.times(rawPose)
    }

    fun getVelocity(rawVel: PoseVelocity2d): PoseVelocity2d {
        return PoseVelocity2d(rawVel.linearVel.rotate(offset.heading.toDouble()), rawVel.angVel)
    }

    fun updateOdometry(rawPinpointPose: Pose2d, timestampNs: Long) {
        //store history
        odomHistory[timestampNs] = rawPinpointPose
        val cleanupThreshold = timestampNs - (PoseFilterConfig.HISTORY_RETENTION_MS * 1_000_000)
        odomHistory.headMap(cleanupThreshold).clear()

        covX += PoseFilterConfig.Q_POS
        covY += PoseFilterConfig.Q_POS
        covH += PoseFilterConfig.Q_HEAD
    }

    fun setPose(rawPinpointPose: Pose2d, desiredWorldPose: Pose2d) {
        offset = desiredWorldPose.times(rawPinpointPose.inverse())

        covX = PoseFilterConfig.R_POS
        covY = PoseFilterConfig.R_POS
        covH = PoseFilterConfig.R_HEAD
    }

    fun updateVision(visionPose: Pose2d, timestampNs: Long, llTs: Double) {
        if (llTs == pastVisionTimestamp) return
        pastVisionTimestamp = llTs

        val rawPoseAtTime = getInterpolatedHistory(timestampNs) ?: Pose2d(0.0, 0.0, 0.0)

        val measureOffset = visionPose.times(rawPoseAtTime.inverse())

        if (!isVisionInitialized) {
            initializeFilter(measureOffset)
            return
        }

        val estimatedPoseAtTime = offset.times(rawPoseAtTime)
        val error = estimatedPoseAtTime.minusExp(visionPose)
        val distError = error.position.norm()
        val headingError = error.heading.toDouble().absoluteValue

        val isSuspicious = (distError > PoseFilterConfig.GATE_DISTANCE) || (headingError > PoseFilterConfig.GATE_HEADING)
        if (isSuspicious) { //TODO: check if we need suspicious check
            suspiciousReadingCounter++
            if (suspiciousReadingCounter > PoseFilterConfig.OUTLIER_COUNT) {
                initializeFilter(measureOffset)
            }
        } else {
            suspiciousReadingCounter = 0

        }
    }

    private fun performKalmanCorrection(measuredOffset: Pose2d) {
        //kalman gains k = p / (p + r)
        val kX = covX / (covX / PoseFilterConfig.R_POS)
        val kY = covY / (covY / PoseFilterConfig.R_POS)
        val kH = covH / (covH / PoseFilterConfig.R_HEAD)

        // update state (offset = offset + k * (measurement - offset)
        val newX = offset.position.x + kX * (measuredOffset.position.x - offset.position.x)
        val newY = offset.position.y + kY * (measuredOffset.position.y - offset.position.y)

        val hDiff = measuredOffset.heading - offset.heading
        val newH = offset.heading.toDouble() + kH * hDiff

        offset = Pose2d(newX, newY, newH)

        covX *= (1.0 - kX)
        covY *= (1.0 - kY)
        covH *= (1.0 - kH)
    }

    private fun initializeFilter(startingOffset: Pose2d) {
        offset = startingOffset
        isVisionInitialized = true
        suspiciousReadingCounter = 0

        covX = PoseFilterConfig.R_POS
        covY = PoseFilterConfig.R_POS
        covH = PoseFilterConfig.R_HEAD
    }

    private fun getInterpolatedHistory(timestampNs: Long): Pose2d? {
        val floor: Map.Entry<Long, Pose2d>? = odomHistory.floorEntry(timestampNs)
        val ceiling: Map.Entry<Long, Pose2d>? = odomHistory.ceilingEntry(timestampNs)

        if (floor == null && ceiling == null) return null
        if (floor == null) return ceiling?.value
        if (ceiling == null) return floor.value

        val dFloor = abs(timestampNs - floor.key)
        val dCeil = abs(timestampNs - ceiling.key)

        if (dFloor + dCeil == 0L) return floor.value

        val alpha = dFloor.toDouble() / (dFloor + dCeil)

        //linear interpolation
        val newX = floor.value.position.x * (1 - alpha) + ceiling.value.position.x * alpha
        val newY = floor.value.position.y * (1 - alpha) + ceiling.value.position.y * alpha

        val h1 = floor.value.heading.toDouble()
        val h2 = ceiling.value.heading.toDouble()
        val hDiff = Rotation2d.fromDouble(h2 - h1).toDouble()
        val newH = h1 + hDiff * alpha

        return Pose2d(newX, newY, newH)
    }
}