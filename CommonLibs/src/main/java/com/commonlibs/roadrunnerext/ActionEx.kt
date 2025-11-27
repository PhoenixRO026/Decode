package com.commonlibs.roadrunnerext

import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.SequentialAction
import com.commonlibs.units.Duration
import com.commonlibs.units.SleepAction

fun Action.delayedBy(duration: Duration) = SequentialAction(SleepAction(duration), this)