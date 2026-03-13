package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.Vector2d
import com.commonlibs.robotWorldPosToTurretWorldPos
import com.commonlibs.robotWorldvelToTurretWorldVel
import com.commonlibs.units.radsec
import com.commonlibs.units.rpm
import org.firstinspires.ftc.teamcode.library.interpolation.InterpolatingTreeMap
import org.firstinspires.ftc.teamcode.library.interpolation.MathUtil
import kotlin.math.cos


class ShootWhileMoving {
    @Config("ShootWhileMovingConfig")
    companion object {
        @JvmField var latencyCompensationSec: Double = 0.1
    }

    data class ShooterParams(
        val rpm: Double,
        val timeOfFlight: Double
    )  {
        companion object {
            fun interpolate(startValue: ShooterParams, endValue: ShooterParams, t: Double): ShooterParams {
                return ShooterParams(
                    MathUtil.interpolate(startValue.rpm, endValue.rpm, t),
                    MathUtil.interpolate(startValue.timeOfFlight, endValue.timeOfFlight, t),
                )
            }
        }
    }

    data class ShooterCommand(
        val targetRpm: Double,
        val targetAngle: Rotation2d
    )

    val shooterTable = InterpolatingTreeMap<Double, ShooterParams>(
        MathUtil::inverseInterpolate,
        ShooterParams::interpolate
    ).apply {
        val cmPerInch = 2.54
        put(177.0 / cmPerInch, ShooterParams(2775.0, 0.422))
        put(202.0 / cmPerInch, ShooterParams(2875.0, 0.477))
        put(227.0 / cmPerInch, ShooterParams(2800.0, 0.535))
        put(252.0 / cmPerInch, ShooterParams(2900.0, 0.607))
        put(277.0 / cmPerInch, ShooterParams(3000.0, 0.642))
        put(302.0 / cmPerInch, ShooterParams(3050.0, 0.732))
        put(327.0 / cmPerInch, ShooterParams(3150.0, 0.762))
        put(352.0 / cmPerInch, ShooterParams(3250.0, 0.880))
        put(377.0 / cmPerInch, ShooterParams(3400.0, 0.892))
        put(402.0 / cmPerInch, ShooterParams(3500.0, 0.948))
    }

    fun calculate(
        robotPosition: Pose2d,
        robotVelocity: PoseVelocity2d,
        goalPos: Vector2d
    ): ShooterCommand {
        val turretPosition = robotWorldPosToTurretWorldPos(robotPosition)
        val turretVelocity = robotWorldvelToTurretWorldVel(robotVelocity, robotPosition.heading)

        // 1. Project future position
        val futurePos = turretPosition + turretVelocity * latencyCompensationSec

        // 2. Get target vector

        val toGoal = goalPos - futurePos
        val distance = toGoal.norm()
        val targetDirection = toGoal / distance

        // 3. Look up baseline velocity from table
        val baseline = shooterTable.get(distance)
        val baselineVelocity = distance / baseline.timeOfFlight

        // 4. Build target velocity vector
        val targetVelocity = targetDirection * baselineVelocity

        // 5. THE MAGIC: subtract robot velocity
        val shotVelocity = targetVelocity - turretVelocity

        // 6. Extract results
        val requiredAngle = shotVelocity.angleCast()
        val requiredVelocity = shotVelocity.norm()

        // 7. Use table in reverse: velocity → effective distance → RPM
        val effectiveDistance = velocityToEffectiveDistance(requiredVelocity)
        val requiredRpm = shooterTable.get(effectiveDistance).rpm

        return ShooterCommand(requiredRpm, requiredAngle)
    }

    fun rpmToHorizontalVelocity(rpm: Double): Double {
        val radPerSec = rpm.rpm.asRadSec
        val tangentialVelocity = radPerSec * (48.0 / 25.4)
        return tangentialVelocity * cos(Math.toRadians(47.2))
    }

    fun horizontalVelocityToRpm(horizVel: Double): Double {
        val tangentialVelocity = horizVel / cos(Math.toRadians(47.2))
        val angularVelocity = tangentialVelocity / (48.0 / 25.40)
        return angularVelocity.radsec.asRpm
    }

    fun velocityToEffectiveDistance(velocity: Double): Double {
        var previousEntry: Map.Entry<Double, ShooterParams>? = null
        for (entry in shooterTable.entrySet()) {
            val dist: Double = entry.key
            val vel: Double = dist / entry.value.timeOfFlight
            if (vel >= velocity) {
                previousEntry?.let { prevEntry ->
                    val prevDist = prevEntry.key
                    val prevVel = prevDist / prevEntry.value.timeOfFlight
                    val t = (velocity - prevVel) / (vel - prevVel)
                    return MathUtil.interpolate(prevDist, dist, t)
                }
                return dist // Interpolate for better accuracy
            }
            previousEntry = entry
        }
        return previousEntry?.key ?: 0.0
    }

}