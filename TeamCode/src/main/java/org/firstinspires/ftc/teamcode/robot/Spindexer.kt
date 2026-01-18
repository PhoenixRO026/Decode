package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.Encoder
import com.commonlibs.units.Duration
import com.commonlibs.units.SleepAction
import com.commonlibs.units.s
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import kotlin.math.abs

class Spindexer(
    val servoTransfer1: Servo,
    val servoTransfer2: Servo,
    val finger: Servo
)
{
    @Config
    data object TransferConfig {
        @JvmField val fingerUpPosition = 0.5
        @JvmField val fingerDownPosition = 0.8


    }

    enum class TransferPos (val pos : Double) {
        intake1(0.5),

        intake2(0.5),

        intake3(0.5),

        shoot1(0.5),

        shoot2(0.5),

        shoot3(0.5),
    }

    var currentPos = TransferPos.intake1

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

    fun goToPos(pos : TransferPos) {
        transferPos = pos.pos
    }

    fun goToNextShoot(pos : TransferPos) {
        if(pos == TransferPos.shoot1) {
            goToPos(TransferPos.shoot2)
        }
        else if(pos == TransferPos.shoot2) {
            goToPos(TransferPos.shoot3)
        }
        else {
            goToPos(TransferPos.shoot1)
        }
    }

    fun goToNextIntake(pos : TransferPos) {
        if(pos == TransferPos.intake1) {
            goToPos(TransferPos.intake2)
        }
        else if(pos == TransferPos.intake2) {
            goToPos(TransferPos.intake3)
        }
        else {
            goToPos(TransferPos.intake1)
        }
    }

    fun goToPosAction(pos : TransferPos) = ParallelAction(
        InstantAction{ goToPos(pos) },
        InstantAction { updateCurrentPos()}
    )
    fun goToNextShootAction() = ParallelAction(
        InstantAction{ goToNextShoot(currentPos) },
        InstantAction { updateCurrentPos()}
    )

    fun goToNextIntakeAction() = ParallelAction(
        InstantAction{ goToNextIntake(currentPos) },
        InstantAction { updateCurrentPos()}
    )


    fun fingerUp() {
        fingerPosition = TransferConfig.fingerUpPosition
    }
    fun fingerDown() {
        fingerPosition = TransferConfig.fingerDownPosition
    }

    fun updateCurrentPos() {
        currentPos = when(transferPos) {
            TransferPos.intake1.pos -> TransferPos.intake1
            TransferPos.intake2.pos -> TransferPos.intake2
            TransferPos.intake3.pos -> TransferPos.intake3
            TransferPos.shoot1.pos -> TransferPos.shoot1
            TransferPos.shoot2.pos -> TransferPos.shoot2
            TransferPos.shoot3.pos -> TransferPos.shoot3
            else -> TransferPos.intake1
        }
    }

    fun shootAction() = SequentialAction(
        InstantAction { fingerUp() },
        SleepAction(0.5.s),
        InstantAction { fingerDown() },
        SleepAction(0.3.s)
    )

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("spindexer pos", transferPos)
        telemetry.addData("finger pos", fingerPosition)
        //telemetry.addData("lift current", rightMotor.getCurrent(CurrentUnit.AMPS) + leftMotor.getCurrent(CurrentUnit.AMPS))
    }
}