@file:Suppress("unused")

package com.commonlibs.units

data class AngularAcceleration(@JvmField var angularVelocity: AngularVelocity, @JvmField var duration: Duration) {
    val asRadSec2 get() = this / (RAD / S / S)
    val asDegSec2 get() = this / (DEG / S / S)
    val asRevSec2 get() = this / (REV / S / S)
    val asRpmSec get() = this / (REV / MIN / S)

    operator fun plus(other: AngularAcceleration) = AngularAcceleration(angularVelocity + other.angularVelocity * (duration / other.duration), duration)
    operator fun minus(other: AngularAcceleration) = AngularAcceleration(angularVelocity - other.angularVelocity * (duration / other.duration), duration)
    operator fun times(scalar: Number) = AngularAcceleration(angularVelocity * scalar, duration)
    operator fun times(otherDuration: Duration) = angularVelocity * (otherDuration / duration)
    operator fun div(scalar: Number) = AngularAcceleration(angularVelocity / scalar, duration)
    operator fun div(other: AngularVelocity) = duration * (other / angularVelocity)
    operator fun div(other: AngularAcceleration) = (angularVelocity / other.angularVelocity) / (duration / other.duration)
    operator fun unaryMinus() = AngularAcceleration(-angularVelocity, duration)

    override fun toString() = "$angularVelocity per " + if (duration.asS == 1.0) "second" else "$duration"
}

val Number.radsec2 get() = this.rad / S / S
val Number.degsec2 get() = this.deg / S / S
val Number.revsec2 get() = this.rev / S / S
val Number.rpmsec get() = this.rev / MIN / S

fun radsec2(number: Double) = number.radsec
fun degsec2(number: Double) = number.degsec
fun revsec2(number: Double) = number.revsec
fun rpmsec(number: Double) = number.rpm

@JvmField val RADSEC2 = 1.radsec
@JvmField val DEGSEC2 = 1.degsec
@JvmField val REVSEC2 = 1.revsec
@JvmField val RPMSEC = 1.rpm