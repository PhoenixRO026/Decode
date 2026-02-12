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
        @JvmField val fingerUpPosition = 0.6
        @JvmField val fingerDownPosition = 0.9
        @JvmField val shootOffset = 0.07
    }

    enum class BallColor {
        PURPLE,
        GREEN,
        EMPTY
    }


    enum class TransferPos (val pos : Double) {
        intake0(0.0256),
        intake1( 0.2189),
        intake2(0.4056),
        shoot0(0.3056),
        shoot1( 0.5039),
        shoot2(0.1139),
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
        if(slots[pos.ordinal] == target) {
            return pos
        }
        else {
            if (pos == TransferPos.shoot0){
                if (slots[pos.ordinal + 1] == target)
                    return TransferPos.shoot1
                else if (slots[pos.ordinal - 1] == target)
                    return TransferPos.shoot2
            }
            else if (pos == TransferPos.shoot1){
                if (slots[pos.ordinal + 1] == target)
                    return TransferPos.shoot2
                else if (slots[pos.ordinal - 1] == target)
                    return TransferPos.shoot0
            }
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


    private fun updateSlot(pos: TransferPos, color: BallColor) {
        slots[pos.ordinal] = color
    }

    private fun emptySlot(pos: TransferPos) {
        slots[pos.ordinal] = BallColor.EMPTY
    }


    fun updateBallSlot() {
        updateHue()
        updateSlot(currentPos, sensorColor)
    }

    fun goToPos(pos : TransferPos) {
        currentPos = pos
    }

    fun goToNextShoot() {
        if (currentPos == TransferPos.shoot0) {
            goToPos(TransferPos.shoot1)
        }
        else if (currentPos == TransferPos.shoot1) {
            goToPos(TransferPos.shoot2)
        }
        else {
            goToPos(TransferPos.shoot0)
        }
    }

    fun goToNextIntake() {
        if (currentPos == TransferPos.intake0) {
            goToPos(TransferPos.intake1)
        }
        else if(currentPos == TransferPos.intake1) {
            goToPos(TransferPos.intake2)
        }
        else {
            goToPos(TransferPos.intake0)
        }
    }

    fun goToPosAction(pos : TransferPos) = InstantAction { goToPos(pos) }

    fun goToNextShootAction() = InstantAction{ goToNextShoot() }

    fun goToNextIntakeAction() = InstantAction{ goToNextIntake() }

    var sensorHue: Float = 0f

    var hsv = floatArrayOf(0f, 0f, 0f)

    val sensorColor get() = when {
        hsv[1] != 0f && sensorHue in 190f..300f -> BallColor.PURPLE
        hsv[1] != 0f && sensorHue in 100f..175f -> BallColor.GREEN
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
        waitForColorAction(BallColor.PURPLE),
        waitForColorAction(BallColor.GREEN),
        SleepAction(duration)
    )

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("spindexer pos", currentPos)
        telemetry.addData("finger pos", fingerPosition)
        //telemetry.addData("lift current", rightMotor.getCurrent(CurrentUnit.AMPS) + leftMotor.getCurrent(CurrentUnit.AMPS))
    }
}