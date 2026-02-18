package com.commonlibs.roadrunnerext

import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.SequentialAction
import com.commonlibs.units.Duration
import com.commonlibs.units.SleepAction

fun Action.delayedBy(duration: Duration) = SequentialAction(SleepAction(duration), this)
@Suppress("FunctionName")
fun ActionWithInit(init: (p: TelemetryPacket) -> Unit, run: (p: TelemetryPacket) -> Boolean) = object : Action {
    var isInit = true
    override fun run(p: TelemetryPacket): Boolean {
        if (isInit) {
            isInit = false
            init(p)
        }
        return run(p)
    }
}