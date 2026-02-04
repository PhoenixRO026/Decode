package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.ftc.Encoder
import com.commonlibs.units.Duration
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import kotlin.math.abs

class Shooter(
    val motorTop: DcMotorEx,
    val motorBottom: DcMotorEx,
    val motorTurret: DcMotorEx,
    val encoderTurret: Encoder,
    val encoderOuttake: Encoder,
    val voltageSensor: VoltageSensor
)
{
    @Config
    data object ShooterConfig {
        @JvmField
        var controllerRpm = PIDController(
            kP = 0.002,
            kD = 0.00004,
            kI = 0.018,
            stabilityThreshold = 50.0
        )
        @JvmField var targetRpmTolerance = 50

        @JvmField
        var kS = 0.8
        @JvmField
        var kV = 0.002146

        @JvmField
        var controllerTurret = PIDController(
            kP = 0.01,
            kD = 0.00025,
            kI = 0.005,
            stabilityThreshold = 0.2
        )
        @JvmField
        var targetPosTolerance = 20
    }

    val rpm get() = encoderOuttake.getPositionAndVelocity().velocity / 28.0 * 60

    var targetRpm = 0.0

    var _powerShooter
        get() = motorTop.power
        set(value) {
            motorTop.power = value
            motorBottom.power = value
        }


    var powerShooter
        get() = _powerShooter
        set(value) {
            if (targetRpm == 0.0)
                _powerShooter = 0.0
            else {
                _powerShooter = value.coerceIn(-1.0, 1.0)
            }
        }

    var powerTurret = motorTurret.power

    private var offset = 0

    val position get() = encoderTurret.getPositionAndVelocity().position - offset

    var targetPos = 0.0

    fun goToRmp(rpm : Double) {
        targetRpm = rpm
    }

    fun goToRpmAction(rpm: Double) = object : Action {
        var init = true
        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                goToRmp(rpm)
            }
            return abs(targetRpm - rpm) > ShooterConfig.targetRpmTolerance
        }
    }

    fun updateRpm(deltaTime: Duration) {
        val voltage = voltageSensor.voltage

        val pidPower = ShooterConfig.controllerRpm.calculate(rpm, targetRpm, deltaTime)

        val feedforwardPower = ShooterConfig.kS + ShooterConfig.kV * targetRpm

        powerShooter = pidPower + feedforwardPower / voltage
    }

    fun goToPos(pos: Double) {
        targetPos = pos
    }

    fun turretToPosAction(pos: Double) = object : Action {
        var init = true
        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                targetPos = pos
            }
            p.addLine("waiting for turret")
            return abs(targetPos - position) > ShooterConfig.targetPosTolerance
        }
    }

    fun updatePos(deltaTime: Duration) {
        powerTurret = ShooterConfig.controllerTurret.calculate(position, targetPos, deltaTime)
    }

    fun update(deltaTime: Duration) {
        updatePos(deltaTime)
        updateRpm(deltaTime)
    }


    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("Outtake power", powerShooter)
        telemetry.addData("Outtake rpm", rpm)
        telemetry.addData("Turret power", powerTurret)
        telemetry.addData("Turret pos", position)

    }
}