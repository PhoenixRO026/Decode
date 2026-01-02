package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
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
    val motor: DcMotorEx,
    val encoder: Encoder,
    val finger: Servo
)
{
    @Config
    data object transferConfig {
        @JvmField
        var controller = PIDController(
            kP = 0.003,
            kD = 0.0001,
            kI = 0.02,
            stabilityThreshold = 50.0
        )

        @JvmField
        val fingerUpPosition = 0.5
        @JvmField
        val fingerDownPosition = 0.95
    }

    val position get() = encoder.getPositionAndVelocity().position

    var fingerPosition : Double = 0.5
        get() = finger.position
        set(value) {
            val clampedVal = value.coerceIn(0.0, 1.0)
            //
            // if (clampedVal == field) return
            field = clampedVal
            finger.position = field
        }

    var power: Double
        get() = motor.power
        set(value) {
            motor.power = value.coerceIn(-1.0, 1.0)
        }

    fun fingerUp() {
        fingerPosition = transferConfig.fingerUpPosition
    }
    fun fingerDown() {
        fingerPosition = transferConfig.fingerDownPosition
    }

    fun shootAction() = SequentialAction(
        InstantAction { fingerUp() },
        SleepAction(0.5.s),
        InstantAction {fingerDown()},
        SleepAction(0.5.s)
    )
    var targetPosition : Double = position


    fun goToPos(pos: Double, multiplier: Double = 1.0, offset: Double= 0.0) {
        targetPosition = pos * multiplier + offset
    }

    fun goToPosAction(pos: Double, multiplier: Double = 1.0, offset: Double = 1.0) = object : Action {
        var init = true
        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                goToPos(pos, multiplier, offset)
            }
            p.addLine("waiting for spindexer")
            return abs(targetPosition - position) > 5
        }
    }

    fun update(deltaTime: Duration) {
        power = transferConfig.controller.calculate(
            position,
            targetPosition,
            deltaTime
        )
    }

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("transfer power", power)
        telemetry.addData("spindexer pos", motor.currentPosition)
        telemetry.addData("finger pos", fingerPosition)
        //telemetry.addData("lift current", rightMotor.getCurrent(CurrentUnit.AMPS) + leftMotor.getCurrent(CurrentUnit.AMPS))
    }
}