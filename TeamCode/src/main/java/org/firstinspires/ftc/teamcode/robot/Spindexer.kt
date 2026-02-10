package org.firstinspires.ftc.teamcode.robot

import android.graphics.Color
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Action
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
        @JvmField val fingerDownPosition = 0.8
        @JvmField val shootOffset = 0.07
    }

    enum class BallColor {
        PURPLE,
        GREEN,
        EMPTY
    }

    val intakePositions = listOf(
        0.15, // slot 0 intake position
        0.50, // slot 1 intake position
        0.85  // slot 2 intake position
    )

    enum class SensorColor {
        PURPLE,
        GREEN,
        NONE
    }

    val slots: MutableList<BallColor> = mutableListOf(
        BallColor.EMPTY,
        BallColor.EMPTY,
        BallColor.EMPTY
    )

    var currentPosition: Double
        get() = servoTransfer1.position
        set(value) {
            servoTransfer1.position = value
            servoTransfer2.position = value
        }

    var activeIntakeSlot: Int? = null


    fun closestSlotToShoot(target: BallColor): Int? {
        if (slots[activeIntakeSlot!!] == target) {
            return activeIntakeSlot!!
        }
        val nextSlot = (activeIntakeSlot!! + 1) % 3
        val previousSlot = (activeIntakeSlot!! + 2) % 3

        if (slots[nextSlot] == target) {
            return nextSlot
        }
        if (slots[previousSlot] == target) {
            return previousSlot
        }
        return null
    }

    fun closestSlotToIntake(): Int? {
        val target = BallColor.EMPTY
        if (slots[activeIntakeSlot!!] == BallColor.EMPTY) {
            return activeIntakeSlot!!
        }
        val nextSlot = (activeIntakeSlot!! + 1) % 3
        val previousSlot = (activeIntakeSlot!! + 2) % 3

        if (slots[nextSlot] == target) {
            return nextSlot
        }
        if (slots[previousSlot] == target) {
            return previousSlot
        }
        return null
    }

    fun shootPositionForSlot(slotIndex: Int): Double {
        return intakePositions[slotIndex] + TransferConfig.shootOffset
    }

    fun goToShootSlot(slotIndex: Int) {
        currentPosition = shootPositionForSlot(slotIndex)
    }


    fun goToIntakeSlot(slotIndex: Int) {
        currentPosition = intakePositions[slotIndex]
        activeIntakeSlot = slotIndex
    }

    fun storeBall(slotIndex: Int, color: BallColor) {
        slots[slotIndex] = color
    }

    fun eraseBall(slotIndex: Int) {
        slots[slotIndex] = BallColor.EMPTY
    }

    private var lastSensorColor: SensorColor = SensorColor.NONE

    fun updateFromColorSensor() {
        val slot = activeIntakeSlot ?: return

        updateHue()
        val current = sensorColor

        val newBall =
            lastSensorColor == SensorColor.NONE &&
                    (current == SensorColor.GREEN || current == SensorColor.PURPLE)

        if (newBall && slots[slot] == BallColor.EMPTY) {
            slots[slot] = when (current) {
                SensorColor.GREEN -> BallColor.GREEN
                SensorColor.PURPLE -> BallColor.PURPLE
                else -> BallColor.EMPTY
            }
        }

        lastSensorColor = current
    }

    fun closestSlotToShootAny(): Int? {
        return closestSlotToShoot(BallColor.GREEN)
            ?: closestSlotToShoot(BallColor.PURPLE)
    }

    var sensorHue: Float = 0f

    var hsv = floatArrayOf(0f, 0f, 0f)

    val sensorColor get() = when {
        hsv[1] != 0f && sensorHue in 0f..40f -> SensorColor.PURPLE
        hsv[1] != 0f && sensorHue in 200f..280f -> SensorColor.GREEN
        hsv[1] != 0f && sensorHue in 40f..110f -> SensorColor.NONE
        else -> SensorColor.NONE
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

    var transferPos
        get() = servoTransfer1.position
        set(value) {
            servoTransfer1.position = value
            servoTransfer2.position = value
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
    fun waitForColorAction(waitColor: SensorColor, maxTime: Duration = 1.s) = RaceAction(
        Action {
            updateHue()
            it.addLine("Waiting for $waitColor")
            sensorColor != waitColor
        },
        SleepAction(maxTime)
    )

    fun waitForColors(duration: Duration) = RaceAction(
        waitForColorAction(SensorColor.PURPLE),
        waitForColorAction(SensorColor.GREEN),
        SleepAction(duration)
    )

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("spindexer pos", transferPos)
        telemetry.addData("finger pos", fingerPosition)
        //telemetry.addData("lift current", rightMotor.getCurrent(CurrentUnit.AMPS) + leftMotor.getCurrent(CurrentUnit.AMPS))
    }
}