@file:Suppress("unused")

package com.commonlibs.units

import com.acmerobotics.roadrunner.Rotation2d
import kotlin.math.PI

data class Angle(@JvmField var asDeg: Double) {
    val asRev get() = asDeg.degToRev()
    val asRad get() = asDeg.degToRad()

    val rotation2d get() = Rotation2d.exp(asRad)

    operator fun plus(other: Angle) = Angle(asDeg + other.asDeg)
    operator fun minus(other: Angle) = Angle(asDeg - other.asDeg)
    operator fun times(scalar: Number) = Angle(asDeg * scalar.toDouble())
    operator fun div(scalar: Number) = Angle(asDeg / scalar.toDouble())
    operator fun div(other: Angle) = asDeg / other.asDeg
    operator fun div(duration: Duration) = AngularVelocity(this, duration)
    operator fun compareTo(other: Angle) = asDeg.compareTo(other.asDeg)
    operator fun unaryMinus() = Angle(-asDeg)

    override fun toString() = "$asDeg " + if (asDeg == 1.0) "degree" else "degrees"

    fun coerceIn(minAng: Angle, maxAng: Angle) = Angle(asDeg.coerceIn(minAng.asDeg, maxAng.asDeg))
}

fun Number.degToRev() = toDouble() / 360.0
fun Number.degToRad() = toDouble() / 180.0 * PI
fun Number.revToDeg() = toDouble() * 360.0
fun Number.revToRad() = toDouble() * 2 * PI
fun Number.radToDeg() = toDouble() * 180 / PI
fun Number.radToRev() = toDouble() / (2 * PI)

val Number.rad get() = Angle(radToDeg())
val Number.deg get() = Angle(toDouble())
val Number.rev get() = Angle(revToDeg())

fun rad(number: Double) = number.rad
fun deg(number: Double) = number.deg
fun rev(number: Double) = number.rev

@JvmField val RAD = 1.rad
@JvmField val DEG = 1.deg
@JvmField val REV = 1.rev

val Rotation2d.angle get() = Angle(toDouble().radToDeg())

fun sin(angle: Angle) = kotlin.math.sin(angle.asRad)
fun cos(angle: Angle) = kotlin.math.cos(angle.asRad)
fun abs(angle: Angle) = Angle(kotlin.math.abs(angle.asDeg))