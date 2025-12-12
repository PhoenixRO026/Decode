package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ftc.Encoder
import com.commonlibs.units.Duration
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
            kP = 0.0015,
            kD = 0.000001,
            kI = 0.0045,
            stabilityThreshold = 50.0
        )
    }

    private var offset = 0.0

    val position get() = encoder.getPositionAndVelocity().position - offset

    var fingerPosition by finger::position

    var power: Double
        get() = motor.power
        set(value) {
            motor.power = value.coerceIn(-1.0, 1.0)
        }

    var targetPosition = position

    fun goToPosAction(pos: Double) = object : Action {
        var init = true
        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                targetPosition = pos
            }
            p.addLine("waiting for spindexer")
            return abs(targetPosition - position) > 5
        }
    }

    fun resetExtendoPosition() {
        offset = encoder.getPositionAndVelocity().position
    }

    fun update(deltaTime: Duration) {
        power = transferConfig.controller.calculate(
            position.toDouble(),
            targetPosition.toDouble(),
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