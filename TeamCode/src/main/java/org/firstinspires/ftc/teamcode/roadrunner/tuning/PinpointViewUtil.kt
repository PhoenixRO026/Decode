package org.firstinspires.ftc.teamcode.roadrunner.tuning

import com.acmerobotics.roadrunner.ftc.PinpointView
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair
import com.acmerobotics.roadrunner.ftc.RollingThreeMedian
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.EncoderDirection
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.teamcode.roadrunner.PinpointLocalizer

fun makePinpointView(pl: PinpointLocalizer): PinpointView = object : PinpointView {
    private val ticks = 19.89436789 //ticks-per-mm for the goBILDA 4-Bar Pod

    private val encoderX get() = pl.driver.encoderX / ticks
    private val encoderY get() = pl.driver.encoderY / ticks

    private var parEncDir: EncoderDirection = pl.initialParDirection
    private var perpEncDir: EncoderDirection = pl.initialPerpDirection

    private fun Double.applyDirection(dir: EncoderDirection): Double = when (dir) {
        EncoderDirection.FORWARD -> this
        EncoderDirection.REVERSED -> this
    }

    private var parVelEstimate = RollingThreeMedian()
    private var perpVelEstimate = RollingThreeMedian()

    private var parVel = 0.0
    private var perpVel = 0.0

    private var parLastPos = encoderX
    private var perpLastPos = encoderY

    private val parPosVel get() = PositionVelocityPair(
        encoderX.applyDirection(parEncDir),
        parVel.applyDirection(parEncDir),
        encoderX,
        parVel
    )

    private val perpPosVel get() = PositionVelocityPair(
        encoderY.applyDirection(perpEncDir),
        perpVel.applyDirection(perpEncDir),
        encoderY,
        perpVel
    )

    private var deltaTime = ElapsedTime()

    private var firstUpdate = true

    override var parDirection: DcMotorSimple.Direction
        get() = when (parEncDir) {
            EncoderDirection.FORWARD -> DcMotorSimple.Direction.FORWARD
            EncoderDirection.REVERSED -> DcMotorSimple.Direction.REVERSE
        }
        set(value) {
            parEncDir = when (value) {
                DcMotorSimple.Direction.FORWARD -> EncoderDirection.FORWARD
                DcMotorSimple.Direction.REVERSE -> EncoderDirection.REVERSED
            }
            pl.driver.setEncoderDirections(parEncDir, perpEncDir)
        }
    override var perpDirection: DcMotorSimple.Direction
        get() = when (perpEncDir) {
            EncoderDirection.FORWARD -> DcMotorSimple.Direction.FORWARD
            EncoderDirection.REVERSED -> DcMotorSimple.Direction.REVERSE
        }
        set(value) {
            perpEncDir = when (value) {
                DcMotorSimple.Direction.FORWARD -> EncoderDirection.FORWARD
                DcMotorSimple.Direction.REVERSE -> EncoderDirection.REVERSED
            }
            pl.driver.setEncoderDirections(parEncDir, perpEncDir)
        }

    override fun getHeadingVelocity(unit: UnnormalizedAngleUnit): Float {
        return pl.driver.getHeadingVelocity(unit).toFloat()
    }

    override fun getParEncoderPositionAndVelocity(): PositionVelocityPair {
        return parPosVel
    }

    override fun getPerpEncoderPositionAndVelocity(): PositionVelocityPair {
        return perpPosVel
    }

    override fun update() {
        pl.driver.update()
        if (firstUpdate) {
            firstUpdate = false
            deltaTime.reset()
            parLastPos = encoderX
            perpLastPos = encoderY
            return
        }
        parVel = parVelEstimate.update((encoderX - parLastPos) / deltaTime.seconds())
        perpVel = perpVelEstimate.update((encoderY - perpLastPos) / deltaTime.seconds())
        parLastPos = encoderX
        perpLastPos = encoderY
        deltaTime.reset()
    }
}