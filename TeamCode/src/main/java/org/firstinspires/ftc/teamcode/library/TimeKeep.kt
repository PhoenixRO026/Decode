package org.firstinspires.ftc.teamcode.library

import com.commonlibs.units.ms
import org.psilynx.psikit.core.Logger

class TimeKeep {
    private var isInitialized = false

    var previousTime = timeNow() - 1.ms
    var currentTime = timeNow()
    inline val deltaTime get() = currentTime - previousTime

    fun resetDeltaTime() {
        if (isInitialized.not()) {
            isInitialized = true
            currentTime = timeNow()
            previousTime = currentTime - 1.ms
            return
        }

        previousTime = currentTime
        currentTime = timeNow()
    }

    private fun timeNow() = (Logger.getTimestamp() / 1_000_000.0).ms
}