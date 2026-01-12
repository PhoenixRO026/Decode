package org.firstinspires.ftc.teamcode.library

import com.commonlibs.units.Time
import com.commonlibs.units.ms
import com.commonlibs.units.s
import org.psilynx.psikit.core.Logger

class TimeKeep {
    private var isInitialized = false

    var previousTime = Logger.getTimestamp().s - 1.ms
    var currentTime = Logger.getTimestamp().s
    inline val deltaTime get() = currentTime - previousTime

    fun resetDeltaTime() {
        if (isInitialized.not()) {
            isInitialized = true
            currentTime = Logger.getTimestamp().s
            previousTime = currentTime - 1.ms
            return
        }

        previousTime = currentTime
        currentTime = Logger.getTimestamp().s
    }
}