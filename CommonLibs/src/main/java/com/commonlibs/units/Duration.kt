@file:Suppress("unused")

package com.commonlibs.units

import com.acmerobotics.roadrunner.SleepAction

class Duration(@JvmField var asS: Double) {
    val asMs get() = asS.sToMs()
    val asMin get() = asS.sToMin()

    operator fun plus(other: Duration) = Duration(asS + other.asS)
    operator fun minus(other: Duration) = Duration(asS - other.asS)
    operator fun times(scalar: Number) = Duration(asS * scalar.toDouble())
    operator fun div(scalar: Number) = Duration(asS / scalar.toDouble())
    operator fun div(other: Duration) = asS / other.asS
    operator fun unaryMinus() = Duration(-asS)
    operator fun compareTo(other: Duration) = asS.compareTo(other.asS)

    override fun toString() = "$asS " + if (asS == 1.0) "second" else "seconds"
}

object Time {
    fun now() = System.currentTimeMillis().ms
}

fun Number.sToMs() = toDouble() * 1000.0
fun Number.sToMin() = toDouble() / 60.0
fun Number.msToS() = toDouble() / 1000.0
fun Number.msToMin() = toDouble() / 60_000.0
fun Number.minToS() = toDouble() * 60.0
fun Number.minToMs() = toDouble() * 60_000.0

val Number.s get() = Duration(toDouble())
val Number.ms get() = Duration(msToS())
val Number.min get() = Duration(minToS())

fun s(number: Double) = number.s
fun ms(number: Double) = number.ms
fun min(number: Double) = number.min

@JvmField val S = 1.s
@JvmField val MS = 1.ms
@JvmField val MIN = 1.min

fun SleepAction(duration: Duration) = SleepAction(duration.asS)