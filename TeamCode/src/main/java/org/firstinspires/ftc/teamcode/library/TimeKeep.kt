package org.firstinspires.ftc.teamcode.library

import com.commonlibs.units.Time
import com.commonlibs.units.ms

class TimeKeep {
    private var isInitialized = false

    var previousTime = Time.now() - 1.ms
    var currentTime = Time.now()
    inline val deltaTime get() = currentTime - previousTime

    fun resetDeltaTime() {
        if (isInitialized.not()) {
            isInitialized = true
            currentTime = Time.now()
            previousTime = currentTime - 1.ms
            return
        }

        previousTime = currentTime
        currentTime = Time.now()
    }
}