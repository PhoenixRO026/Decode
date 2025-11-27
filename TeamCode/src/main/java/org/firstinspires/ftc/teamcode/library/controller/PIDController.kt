package org.firstinspires.ftc.teamcode.library.controller

import com.commonlibs.units.Duration
import com.commonlibs.units.ms
import kotlin.math.absoluteValue
import kotlin.math.sign

/**
 * @param kP proportional term
 * @param kI integral term
 * @param kD derivative term
 * @param integralSumLimit the maximum our integral sum can go to
 *                              (this * Kp should be less than the maximum output of your system)
 * @param newTargetReset should the system reset the integral sum if the target changes
 * @param derivativeFilter low pass filter for the derivative
 * @param stabilityThreshold the maximum our derivative can be before we integrate.
 *                           This ensures we have better stability
 */
@Suppress("unused")
data class PIDController(
    @JvmField var kP: Double = 0.0,
    @JvmField var kI: Double = 0.0,
    @JvmField var kD: Double = 0.0,
    @JvmField var integralSumLimit: Double = 0.0,
    @JvmField var newTargetReset: Boolean = true,
    @JvmField var zeroTargetReset: Boolean = true,
    @JvmField var derivativeFilter: LowPassFilter = LowPassFilter(),
    @JvmField var stabilityThreshold: Double = 0.0,
) {
    private var previousTime = 0.ms
    private var init = true
    private var previousError = 0.0
    private var previousTarget = 0.0

    private var innerIntegralSum = 0.0
    private var innerDerivative = 0.0

    private val derivative by ::innerDerivative
    private val integralSum by ::innerIntegralSum

    val derivativeKD get() = derivative * kD
    val integralSumKI get() = integralSum * kI

    fun calculate(position: Double, target: Double, dt: Duration): Double {
        val error = target - position

        innerDerivative = calculateDerivative(error, dt.asS)

        integrate(error, target, dt.asS, derivative)

        previousTarget = target

        previousError = error

        return error * kP + integralSum * kI + derivative * kD
    }

    private fun integrate(error: Double, target: Double, dt: Double, derivative: Double) {
        if (crossOverDetected(error)) {
            innerIntegralSum = 0.0
        }

        if (newTargetReset && target != previousTarget) {
            innerIntegralSum = 0.0
        }

        if (zeroTargetReset && error == 0.0) {
            innerIntegralSum = 0.0
        }

        if (stabilityThreshold > 0.0 && derivative.absoluteValue > stabilityThreshold) return

        innerIntegralSum += ((error + previousError) / 2) * dt

        if (integralSumLimit > 0) {
            innerIntegralSum = innerIntegralSum.coerceIn(-integralSumLimit, integralSumLimit)
        }
    }

    private fun calculateDerivative(error: Double, dt: Double): Double {
        val derivative = (error - previousError) / dt
        return if (derivative.isFinite())
            derivativeFilter.estimate(derivative)
        else
            0.0
    }

    private fun crossOverDetected(error: Double): Boolean {
        return error.sign != previousError.sign && error.sign != 0.0
    }

    fun resetIntegral() {
        innerIntegralSum = 0.0
    }
}