package org.firstinspires.ftc.teamcode.robot

import android.graphics.Color
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.RaceAction
import com.commonlibs.units.Duration
import com.commonlibs.units.SleepAction
import com.commonlibs.units.dist
import com.commonlibs.units.s
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.psilynx.psikit.core.Logger

class Spindexer(
    val servoTransfer1: Servo,
    val servoTransfer2: Servo,
    val finger: Servo,
    val colorSensor: NormalizedColorSensor
)
{
    @Config
    data object TransferConfig {
        @JvmField val fingerUpPosition = 0.5989
        @JvmField val  fingerDownPosition = 0.4972
        @JvmField val shootOffset = 0.07

        @JvmField val intake0Pos = 0.0
        @JvmField val wholePosDist = 0.0
        @JvmField val toShootPosDist = 0.0
    }

    enum class BallColor {
        PURPLE,
        GREEN,
        EMPTY
    }

    enum class TransferPos (val pos : Double, val index: Int) {
        intake0(TransferConfig.intake0Pos, 0),
        intake1(TransferConfig.intake0Pos + TransferConfig.wholePosDist, 1),
        intake2(TransferConfig.intake0Pos + 2 * TransferConfig.wholePosDist, 2),
        shoot2(TransferConfig.toShootPosDist + TransferConfig.intake0Pos, 2),
        shoot0(TransferConfig.toShootPosDist + TransferConfig.intake0Pos + TransferConfig.wholePosDist, 0),
        shoot1(TransferConfig.toShootPosDist + TransferConfig.intake0Pos + 2 * TransferConfig.wholePosDist, 1),
        pseudo2(TransferConfig.toShootPosDist + TransferConfig.intake0Pos + 3 * TransferConfig.wholePosDist, 2),
        pseudo0(TransferConfig.toShootPosDist + TransferConfig.intake0Pos + 4 * TransferConfig.wholePosDist, 0)
    }

    /*
    * intake0(0.0883, 0),
        intake1(0.2344, 1),
        intake2(0.3661, 2),
        shoot2(0.1661, 2),
        shoot0(0.3039, 0),
        shoot1(0.4339, 1),
        pseudo2(0.5733, 2),
        pseudo0(0.7089, 0)
    * */
    val slots: MutableList<BallColor> = mutableListOf(
        BallColor.EMPTY,
        BallColor.EMPTY,
        BallColor.EMPTY
    )

    var currentPos = TransferPos.intake0
        get() = field
        set(value) {
            val clampedVal = value.pos.coerceAtMost(1.0 - 0.0044)
            servoTransfer1.position = clampedVal
            servoTransfer2.position = clampedVal + 0.0044
            field = value
        }

    var distance = (colorSensor as DistanceSensor).getDistance(DistanceUnit.MM)

    fun closestSlotToShoot(pos: TransferPos,target: BallColor): TransferPos? {
        when (pos) {
            TransferPos.shoot0, TransferPos.shoot1, TransferPos.shoot2 -> {
                if (slots[pos.index] == target) {
                    return pos
                }
            }
            else -> {}
        }

        val lowerPos = findPreviousShootIndex(pos)

        val upperPos = findNextShootIndex(pos)

        if (pos == TransferPos.intake0) {
            if (slots[pos.index] == target) {
                return TransferPos.shoot0
            }

            if (slots[upperPos.index] == target) {
                return upperPos
            }
        }

        if (slots[lowerPos.index] == target) {
            return lowerPos
        }
        if (slots[upperPos.index] == target) {
            return upperPos
        }

        if (slots[pos.index] == target) {
            return when (pos) {
                TransferPos.intake0 -> TransferPos.shoot0
                TransferPos.intake1 -> TransferPos.shoot1
                TransferPos.intake2 -> TransferPos.shoot2
                else -> null
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

    fun goToPurpleAction() = InstantAction {goToPurple()}
    fun goToGreenAction() = InstantAction {goToGreen()}

    private fun updateSlot(pos: TransferPos, color: BallColor) {
        slots[pos.index] = color
    }

    fun emptySlot(pos: TransferPos) {
        slots[pos.index] = BallColor.EMPTY
    }


    fun updateBallSlot() {
        //updateHue()
        updateSlot(currentPos, sensorColor)
    }

    fun goToPos(pos : TransferPos) {
        currentPos = pos
    }

    fun findNextShoot(pos: TransferPos) = when (pos) {
        TransferPos.intake0 -> TransferPos.shoot2
        TransferPos.intake1 -> TransferPos.shoot2
        TransferPos.intake2 -> TransferPos.shoot0
        TransferPos.shoot1 -> TransferPos.pseudo2
        TransferPos.shoot2 -> TransferPos.shoot0
        TransferPos.shoot0 -> TransferPos.shoot1
        TransferPos.pseudo2 -> TransferPos.pseudo0
        TransferPos.pseudo0 -> TransferPos.shoot1
    }

    fun findNextShootIndex(pos: TransferPos) = when (pos) {
        TransferPos.intake0 -> TransferPos.shoot2
        TransferPos.intake1 -> TransferPos.shoot0
        TransferPos.intake2 -> TransferPos.shoot1
        TransferPos.shoot1 -> TransferPos.pseudo2
        TransferPos.shoot2 -> TransferPos.shoot0
        TransferPos.shoot0 -> TransferPos.shoot1
        TransferPos.pseudo2 -> TransferPos.pseudo0
        TransferPos.pseudo0 -> TransferPos.shoot1
    }

    fun findPreviousShootIndex(pos: TransferPos) = when (pos) {
        TransferPos.intake0 -> TransferPos.shoot0
        TransferPos.intake1 -> TransferPos.shoot2
        TransferPos.intake2 -> TransferPos.shoot0
        TransferPos.shoot0 -> TransferPos.shoot2
        TransferPos.shoot2 -> TransferPos.shoot1
        TransferPos.shoot1 -> TransferPos.shoot0
        TransferPos.pseudo2 -> TransferPos.shoot1
        TransferPos.pseudo0 -> TransferPos.pseudo2
    }

    fun goToNextShoot() {
        goToPos(findNextShoot(currentPos))
    }

    fun findNextIntake(pos: TransferPos) = when (pos) {
        TransferPos.intake0 -> TransferPos.intake1
        TransferPos.intake1 -> TransferPos.intake2
        TransferPos.intake2 -> TransferPos.intake0
        TransferPos.shoot0 -> TransferPos.intake0
        TransferPos.shoot1 -> TransferPos.intake0
        TransferPos.shoot2 -> TransferPos.intake0
        TransferPos.pseudo2 -> TransferPos.intake0
        TransferPos.pseudo0 -> TransferPos.intake0
    }

    fun goToNextIntake() {
        goToPos(findNextIntake(currentPos))
    }

    fun goToPosAction(pos : TransferPos) = InstantAction { goToPos(pos) }

    fun goToNextShootAction() = InstantAction{ goToNextShoot() }

    fun goToNextIntakeAction() = InstantAction{
        goToNextIntake()
        Logger.recordOutput("Spindexer/nextIntakeAction", currentPos)
    }

    var sensorHue: Float = 0f

    var hsv = floatArrayOf(0f, 0f, 0f)

    val sensorColor get() = when {
        hsv[1] != 0f && sensorHue in 162f..190f -> BallColor.PURPLE
        hsv[1] != 0f && sensorHue in 151f..161f -> BallColor.GREEN
        else -> BallColor.EMPTY
    }

    fun  updateHue() {
        val normalizedColors = colorSensor.normalizedColors
        Color.RGBToHSV(
            (normalizedColors.red * 256).toInt(),
            (normalizedColors.green * 256).toInt(),
            (normalizedColors.blue * 256).toInt(),
            hsv
        )
        sensorHue = hsv[0]
        Logger.recordOutput("Spindexer/sensorHue", sensorHue)
        Logger.recordOutput("Spindexer/sensorColor", sensorColor)
    }

    fun updateDistance() {
        distance = (colorSensor as DistanceSensor).getDistance(DistanceUnit.MM)
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

    fun waitForDistance(maxTime: Duration = 1.s) = RaceAction(
        {
            updateDistance()
            it.addLine("Waiting for distance")
            distance >= 57.0
        },
        SleepAction(maxTime)
    )

    fun waitForAnyColor(maxTime: Duration = 1.s) = RaceAction(
        {
            updateHue()
            it.addLine("Waiting for any ball")
            Logger.recordOutput("Spindexer/waitForColorResult", sensorColor)
            sensorColor == BallColor.EMPTY
        },
        SleepAction(maxTime)
    )

    fun waitForColors(duration: Duration) = waitForAnyColor(duration)

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