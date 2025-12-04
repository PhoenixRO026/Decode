package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.ftc.Encoder
import com.commonlibs.units.Duration
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.library.controller.PIDController

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
            kP = 0.015,
            kD = 0.00069420,
            kI = 0.001,
            stabilityThreshold = 0.2
        )
        @JvmField var targetPosTolerance = 10
    }

    private var offset = 0

    val position get() = encoder.getPositionAndVelocity().position - offset

    var fingerPosition by finger::position
    var power
        get() = motor.power
        set(value) {
            motor.power = value
        }

    var targetPosition = position
        set(value) {
            field = value
        }

    fun update(deltaTime: Duration) {
        power = transferConfig.controller.calculate(
            position,
            targetPosition,
            deltaTime
        )
    }

    fun open_trap() {
        // activate servo trap to release ball to the shooter
        finger.position = 1.0
    }

    fun close_trap() {
        // closes trap after transferring ball to the shooter
        finger.position = 0.0
    }

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("transfer power", power)
        telemetry.addData("spindexer pos", motor.currentPosition)
        telemetry.addData("finger pos", fingerPosition)
    }
}