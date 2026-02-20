package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.ftc.Encoder
import com.commonlibs.units.Duration
import com.qualcomm.robotcore.hardware.DcMotorEx
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
            kP = 0.0015,
            kD = 0.00015,
            kI = 0.000001,
            stabilityThreshold = 50.0
        )
        @JvmField var targetRpmTolerance = 50

        @JvmField var kS = 0.06
        @JvmField var kV = 0.00303

        @JvmField
        var controllerTurret = PIDController(
            kP = 0.0005,
            kD = 0.00005,
            kI = 0.00045,
            stabilityThreshold = 0.2
        )
        @JvmField var ticksPerRev = 8192.0 * (15.0 / 22.0)

        @JvmField var targetPosTolerance = 3
        @JvmField var minTurretPosition = -14000.0
        @JvmField var maxTurretPosition = 13300.0
        @JvmField var limitTolerence = 5
    }

    val rpm get() = encoderOuttake.getPositionAndVelocity().velocity / 28.0 * 60

    var rpmFar : Double = 3280.0
    var rpmClose : Double = 2830.0

    var shootClosePos : Double = 500.0
    var shootFarPos : Double = 300.0

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

    var powerTurret
        get() = motorTurret.power
        set(value) {
            motorTurret.power = value
        }

    private var offset = 0.0

    val turretPosition get() = encoderTurret.getPositionAndVelocity().position - offset

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
            return abs(targetPos - turretPosition) > ShooterConfig.targetPosTolerance
        }
    }

    fun stopShootAction() = ParallelAction(
        InstantAction { goToRpmAction(0.0)}
    )

    fun updateTurretPos(deltaTime: Duration, error: Double) {
        targetPos += error * ((ShooterConfig.ticksPerRev * (15.0 / 108.0)) / 360.0)
        if (targetPos > ShooterConfig.maxTurretPosition - ShooterConfig.limitTolerence) {
            targetPos = ShooterConfig.minTurretPosition
        }
        if (targetPos < ShooterConfig.minTurretPosition + ShooterConfig.limitTolerence) {
            targetPos = ShooterConfig.maxTurretPosition
        }
        powerTurret = ShooterConfig.controllerTurret.calculate(turretPosition, targetPos, deltaTime)
    }

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("Outtake power", powerShooter)
        telemetry.addData("Outtake rpm", rpm)
        telemetry.addData("Turret power", powerTurret)
        telemetry.addData("Turret pos", turretPosition)
    }
}