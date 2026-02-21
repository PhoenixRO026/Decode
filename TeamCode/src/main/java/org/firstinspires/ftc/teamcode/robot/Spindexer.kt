package org.firstinspires.ftc.teamcode.robot

import android.graphics.Color
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.RaceAction
import com.commonlibs.units.Duration
import com.commonlibs.units.SleepAction
import com.commonlibs.units.s
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry

class Spindexer(
    val servoTransfer1: Servo,
    val servoTransfer2: Servo,
    val finger: Servo,
    val colorSensor: NormalizedColorSensor
)
{
    @Config
    data object TransferConfig {
        @JvmField val fingerUpPosition = 0.5
        @JvmField val fingerDownPosition = 0.9
        @JvmField val shootOffset = 0.07
    }

    enum class BallColor {
        PURPLE,
        GREEN,
        EMPTY
    }


    enum class TransferPos (val pos : Double, val index: Int) {
        intake0(0.0, 0),
        intake1( 0.2056, 1),
        intake2(0.3865, 2),
        shoot0(0.2883, 0),
        shoot1(0.0980, 1),
        shoot2(0.4728, 2),
    }

    val slots: MutableList<BallColor> = mutableListOf(
        BallColor.EMPTY,
        BallColor.EMPTY,
        BallColor.EMPTY
    )

    var currentPos = TransferPos.intake0
        set(value) {
            servoTransfer1.position = value.pos
            servoTransfer2.position = value.pos
            field = value
        }

    var activeIntakeSlot: Int? = null


    fun closestSlotToShoot(pos: TransferPos,target: BallColor): TransferPos? {
        when (pos) {
            TransferPos.shoot0, TransferPos.shoot1, TransferPos.shoot2 -> {
                if (slots[pos.index] == target) {
                    return pos
                }
            }
            else -> {}
        }

        val lowerPos = findPreviousShoot(pos)

        val upperPos = findNextShoot(pos)

        if (slots[lowerPos.index] == target) {
            return lowerPos
        }
        if (slots[upperPos.index] == target) {
            return upperPos
        }

        return null
    }

    fun goToGreen() {
        val target = closestSlotToShoot(currentPos, BallColor.GREEN)
        if (target != null) {
            goToPos(target)
        }
    }

    fun goToPurple() {
        val target = closestSlotToShoot(currentPos, BallColor.PURPLE)
        if (target != null) {
            goToPos(target)
        }
    }

    fun goToPurpleAction() = InstantAction {goToPurple()}
    fun goToGreenAction() = InstantAction {goToGreen()}

    private fun updateSlot(pos: TransferPos, color: BallColor) {
        slots[pos.index] = color
    }

    fun emptySlot(pos: TransferPos) {
        slots[pos.index] = BallColor.EMPTY
    }


    fun updateBallSlot() {
        updateHue()
        updateSlot(currentPos, sensorColor)
    }

    fun goToPos(pos : TransferPos) {
        currentPos = pos
    }

    fun findNextShoot(pos: TransferPos) = when (pos) {
        TransferPos.intake0 -> TransferPos.shoot2
        TransferPos.intake1 -> TransferPos.shoot0
        TransferPos.intake2 -> TransferPos.shoot1
        TransferPos.shoot0 -> TransferPos.shoot1
        TransferPos.shoot1 -> TransferPos.shoot2
        TransferPos.shoot2 -> TransferPos.shoot0
    }

    fun findPreviousShoot(pos: TransferPos) = when (pos) {
        TransferPos.intake0 -> TransferPos.shoot1
        TransferPos.intake1 -> TransferPos.shoot2
        TransferPos.intake2 -> TransferPos.shoot0
        TransferPos.shoot0 -> TransferPos.shoot2
        TransferPos.shoot1 -> TransferPos.shoot0
        TransferPos.shoot2 -> TransferPos.shoot1
    }

    fun goToNextShoot() {
        goToPos(findNextShoot(currentPos))
    }

    fun findNextIntake(pos: TransferPos) = when (pos) {
        TransferPos.intake0 -> TransferPos.intake1
        TransferPos.intake1 -> TransferPos.intake2
        TransferPos.intake2 -> TransferPos.intake0
        TransferPos.shoot0 -> TransferPos.intake2
        TransferPos.shoot1 -> TransferPos.intake0
        TransferPos.shoot2 -> TransferPos.intake1
    }

    fun goToNextIntake() {
        goToPos(findNextIntake(currentPos))
    }

    fun goToPosAction(pos : TransferPos) = InstantAction { goToPos(pos) }

    fun goToNextShootAction() = InstantAction{ goToNextShoot() }

    fun goToNextIntakeAction() = InstantAction{ goToNextIntake() }

    var sensorHue: Float = 0f

    var hsv = floatArrayOf(0f, 0f, 0f)

    val sensorColor get() = when {
        hsv[1] != 0f && sensorHue in 180f..250f -> BallColor.PURPLE
        hsv[1] != 0f && sensorHue in 120f..180f -> BallColor.GREEN
        else -> BallColor.EMPTY
    }

    fun updateHue() {
        val normalizedColors = colorSensor.normalizedColors
        Color.RGBToHSV(
            (normalizedColors.red * 256).toInt(),
            (normalizedColors.green * 256).toInt(),
            (normalizedColors.blue * 256).toInt(),
            hsv
        )
        sensorHue = hsv[0]
    }


    var fingerPosition : Double = 0.5
        get() = finger.position
        set(value) {
            val clampedVal = value.coerceIn(0.0, 1.0)
            field = clampedVal
            finger.position = field
        }

    fun fingerUp() {
        finger.position = TransferConfig.fingerUpPosition
    }

    fun fingerDown() {
        finger.position = TransferConfig.fingerDownPosition
    }

    // waits for specific color
    fun waitForColorAction(waitColor: BallColor, maxTime: Duration = 1.s) = RaceAction(
        Action {
            updateHue()
            it.addLine("Waiting for $waitColor")
            sensorColor != waitColor
        },
        SleepAction(maxTime)
    )

    fun waitForColors(duration: Duration) = RaceAction(
        waitForColorAction(BallColor.PURPLE, duration),
        waitForColorAction(BallColor.GREEN, duration),
        SleepAction(duration)
    )

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("spindexer pos", currentPos)
        telemetry.addData("finger pos", fingerPosition)
        //telemetry.addData("lift current", rightMotor.getCurrent(CurrentUnit.AMPS) + leftMotor.getCurrent(CurrentUnit.AMPS))
    }

    fun init() {
        currentPos = TransferPos.intake0
        fingerDown()
    }
}