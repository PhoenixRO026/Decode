@file:Suppress("unused")

package com.commonlibs.units

data class Acceleration(@JvmField var velocity: Velocity, @JvmField var duration: Duration) {
    val asMSec2 get() = this / (M / S / S)
    val asCmSec2 get() = this / (CM / S / S)
    val asMmSec2 get() = this / (MM / S / S)
    val asInchSec2 get() = this / (INCH / S / S)

    operator fun plus(other: Acceleration) = Acceleration(velocity + other.velocity * (duration / other.duration), duration)
    operator fun minus(other: Acceleration) = Acceleration(velocity - other.velocity * (duration / other.duration), duration)
    operator fun times(scalar: Number) = Acceleration(velocity * scalar, duration)
    operator fun times(otherDuration: Duration) = velocity * (otherDuration / duration)
    operator fun div(scalar: Number) = Acceleration(velocity / scalar, duration)
    operator fun div(other: Velocity) = duration * (other / velocity)
    operator fun div(other: Acceleration) = (velocity / other.velocity) / (duration / other.duration)
    operator fun unaryMinus() = Acceleration(-velocity, duration)

    override fun toString() = "$velocity per " + if (duration.asS == 1.0) "second" else "$duration"
}

val Number.msec2 get() = this.m / S / S
val Number.cmsec2 get() = this.cm / S / S
val Number.mmsec2 get() = this.mm / S / S
val Number.inchsec2 get() = this.inch / S / S

fun msec2(number: Double) = number.msec2
fun cmsec2(number: Double) = number.cmsec2
fun mmsec2(number: Double) = number.mmsec2
fun inchsec2(number: Double) = number.inchsec2

@JvmField val MSEC2 = 1.msec2
@JvmField val CMSEC2 = 1.cmsec2
@JvmField val MMSEC2 = 1.mmsec2
@JvmField val INCHSEC2 = 1.inchsec2