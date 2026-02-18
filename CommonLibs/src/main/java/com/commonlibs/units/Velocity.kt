@file:Suppress("unused")

package com.commonlibs.units

data class Velocity(@JvmField var distance: Distance, @JvmField var duration: Duration) {
    val asMSec get() = this / (M / S)
    val asCmSec get() = this / (CM / S)
    val asMmSec get() = this / (MM / S)
    val asInchSec get() = this / (INCH / S)

    operator fun plus(other: Velocity) = Velocity(distance + other.distance * (duration / other.duration), duration)
    operator fun minus(other: Velocity) = Velocity(distance - other.distance * (duration / other.duration), duration)
    operator fun times(scalar: Number) = Velocity(distance * scalar, duration)
    operator fun times(otherDuration: Duration) = distance * (otherDuration / duration)
    operator fun div(scalar: Number) = Velocity(distance / scalar, duration)
    operator fun div(other: Distance) = duration * (other / distance)
    operator fun div(other: Velocity) = (distance / other.distance) / (duration / other.duration)
    operator fun unaryMinus() = Velocity(-distance, duration)
    operator fun div(duration: Duration) = Acceleration(this, duration)

    override fun toString() = "$distance per " + if (duration.asS == 1.0) "second" else "$duration"
}

val Number.msec get() = this.m / S
val Number.cmsec get() = this.cm / S
val Number.mmsec get() = this.mm / S
val Number.inchsec get() = this.inch / S

fun msec(number: Double) = number.msec
fun cmsec(number: Double) = number.cmsec
fun mmsec(number: Double) = number.mmsec
fun inchsec(number: Double) = number.inchsec

@JvmField val MSEC = 1.msec
@JvmField val CMSEC = 1.cmsec
@JvmField val MMSEC = 1.mmsec
@JvmField val INCHSEC = 1.inchsec