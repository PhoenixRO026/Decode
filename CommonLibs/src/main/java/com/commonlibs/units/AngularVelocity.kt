@file:Suppress("unused")

package com.commonlibs.units

data class AngularVelocity(@JvmField var angle: Angle, @JvmField var duration: Duration) {
    val asRadSec get() = this / (RAD / S)
    val asDegSec get() = this / (DEG / S)
    val asRevSec get() = this / (REV / S)
    val asRpm get() = this / (REV / MIN)

    operator fun plus(other: AngularVelocity) = AngularVelocity(angle + other.angle * (duration / other.duration), duration)
    operator fun minus(other: AngularVelocity) = AngularVelocity(angle - other.angle * (duration / other.duration), duration)
    operator fun times(scalar: Number) = AngularVelocity(angle * scalar, duration)
    operator fun times(otherDuration: Duration) = angle * (otherDuration / duration)
    operator fun div(scalar: Number) = AngularVelocity(angle / scalar, duration)
    operator fun div(other: Angle) = duration * (other / angle)
    operator fun div(other: AngularVelocity) = (angle / other.angle) / (duration / other.duration)
    operator fun unaryMinus() = AngularVelocity(-angle, duration)
    operator fun div(duration: Duration) = AngularAcceleration(this, duration)

    override fun toString() = "$angle per " + if (duration.asS == 1.0) "second" else "$duration"
}

val Number.radsec get() = this.rad / S
val Number.degsec get() = this.deg / S
val Number.revsec get() = this.rev / S
val Number.rpm get() = this.rev / MIN

fun radsec(number: Double) = number.radsec
fun degsec(number: Double) = number.degsec
fun revsec(number: Double) = number.revsec
fun rpm(number: Double) = number.rpm

@JvmField val RADSEC = 1.radsec
@JvmField val DEGSEC = 1.degsec
@JvmField val REVSEC = 1.revsec
@JvmField val RPM = 1.rpm