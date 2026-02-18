package org.firstinspires.ftc.teamcode.library.controller

data class LowPassFilter(
    @JvmField var gain: Double = 0.0
) {
    private var previousEstimate = 0.0
    private var init = true

    fun estimate(measurement: Double): Double {
        if (init) {
            init = false
            previousEstimate = measurement
        }
        val estimate = gain * previousEstimate + (1 - gain) * measurement
        previousEstimate = estimate
        return estimate
    }
}