@file:Suppress("unused")

package com.commonlibs.units

import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

data class Distance(
    @JvmField var asInch: Double
) {
    val asCm get() = asInch.inchToCm()
    val asMm get() = asInch.inchToMm()
    val asM get() = asInch.inchToM()

    val x get() = Distance2d(this, 0.m)
    val y get() = Distance2d(0.m, this)

    operator fun plus(other: Distance) = Distance(asInch + other.asInch)
    operator fun minus(other: Distance) = Distance(asInch - other.asInch)
    operator fun times(scalar: Number) = Distance(asInch * scalar.toDouble())
    operator fun div(scalar: Number) = Distance(asInch / scalar.toDouble())
    operator fun div(other: Distance) = asInch / other.asInch
    operator fun div(duration: Duration) = Velocity(this, duration)
    operator fun unaryMinus() = Distance(-asInch)

    override fun toString() = "$asInch " + if (asInch == 1.0) "inch" else "inches"
}

fun Number.cmToInch() = toDouble() / 2.54
fun Number.cmToM() = toDouble() / 100.0
fun Number.cmToMm() = toDouble() * 10.0
fun Number.inchToCm() = toDouble() * 2.54
fun Number.inchToM() = toDouble() * 0.0254
fun Number.inchToMm() = toDouble() * 25.4
fun Number.mToCm() = toDouble() * 100.0
fun Number.mToInch() = toDouble() / 0.0254
fun Number.mToMm() = toDouble() * 1000.0
fun Number.mmToCm() = toDouble() / 10.0
fun Number.mmToInch() = toDouble() / 25.4
fun Number.mmToM() = toDouble() / 1000.0

val Number.m get() = Distance(mToInch())
val Number.mm get() = Distance(mmToInch())
val Number.cm get() = Distance(cmToInch())
val Number.inch get() = Distance(toDouble())

fun m(number: Double) = number.m
fun mm(number: Double) = number.mm
fun cm(number: Double) = number.cm
fun inch(number: Double) = number.inch

@JvmField val M = 1.m
@JvmField val MM = 1.mm
@JvmField val CM = 1.cm
@JvmField val INCH = 1.inch

@Suppress("FunctionName")
fun Vector2d(x: Distance, y: Distance) = Distance2d(x, y)

fun Vector2d.headingTowards(destination: Vector2d): Pose2d {
    val diffVec = Vector2d(
        x = destination.x - x,
        y = destination.y - y
    )
    return Pose2d(this, atan2(diffVec.y, diffVec.x))
}

data class Distance2d(@JvmField var x: Distance, @JvmField var y: Distance) {
    operator fun plus(other: Distance2d) = Distance2d(x + other.x, y + other.y)
    operator fun minus(other: Distance2d) = Distance2d(x - other.x, y - other.y)
    operator fun times(scalar: Number) = Distance2d(x * scalar, y * scalar)
    operator fun div(scalar: Number) = Distance2d(x / scalar, y / scalar)
    operator fun unaryMinus() = Distance2d(-x, -y)

    val asMeters get() = Vector2d(x.asM, y.asM)
    val asMm get() = Vector2d(x.asMm, y.asMm)
    val asCm get() = Vector2d(x.asCm, y.asCm)
    val asInch get() = Vector2d(x.asInch, y.asInch)

    val vector2d get() = asInch

    fun headingTowards(destination: Distance2d): Pose {
        return this.vector2d.headingTowards(destination.vector2d).pose
    }
}

val Vector2d.m get() = Distance2d(x.m, y.m)
val Vector2d.mm get() = Distance2d(x.mm, y.mm)
val Vector2d.cm get() = Distance2d(x.cm, y.cm)
val Vector2d.inch get() = Distance2d(x.inch, y.inch)
val Vector2d.dist get() = inch

fun m(vector: Vector2d) = vector.m
fun mm(vector: Vector2d) = vector.mm
fun cm(vector: Vector2d) = vector.cm
fun inch(vector: Vector2d) = vector.inch
fun dist(vector: Vector2d) = vector.dist

@Suppress("FunctionName")
fun Pose2d(position: Distance2d, heading: Angle) = Pose(position, heading)

@Suppress("FunctionName")
fun Pose2d(x: Distance, y: Distance, heading: Angle) = Pose(x, y, heading)

data class Pose(@JvmField var position: Distance2d, @JvmField var heading: Angle) {
    constructor(x: Distance, y: Distance, heading: Angle) : this(Distance2d(x, y), heading)

    operator fun plus(other: Pose) = Pose(position + other.position, heading + other.heading)
    operator fun plus(other: Distance2d) = Pose(position + other, heading)
    operator fun plus(other: Angle) = Pose(position, heading + other)
    operator fun minus(other: Pose) = Pose(position - other.position, heading - other.heading)
    operator fun minus(other: Distance2d) = Pose(position - other, heading)
    operator fun minus(other: Angle) = Pose(position, heading - other)

    val pose2d get() = Pose2d(position.asInch, heading.asRad)
}

val Pose2d.m get() = Pose(position.m, heading.angle)
val Pose2d.mm get() = Pose(position.mm, heading.angle)
val Pose2d.cm get() = Pose(position.cm, heading.angle)
val Pose2d.inch get() = Pose(position.inch, heading.angle)
val Pose2d.pose get() = inch

fun m(pose2d: Pose2d) = pose2d.m
fun mm(pose2d: Pose2d) = pose2d.mm
fun cm(pose2d: Pose2d) = pose2d.cm
fun inch(pose2d: Pose2d) = pose2d.inch
fun pose(pose2d: Pose2d) = pose2d.pose

fun Vector2d.rotate(radians: Double) = Vector2d(
    x * cos(radians) - y * sin(radians),
    x * sin(radians) + y * cos(radians)
)

fun Vector2d.rotate(angle: Angle) = Vector2d(
    x * cos(angle) - y * sin(angle),
    x * sin(angle) + y * cos(angle)
)