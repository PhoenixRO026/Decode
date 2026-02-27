package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.ftc.Encoder
import com.commonlibs.units.AngularVelocity
import com.commonlibs.units.Duration
import com.commonlibs.units.deg
import com.commonlibs.units.radsec
import com.commonlibs.units.rev
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import org.firstinspires.ftc.teamcode.robot.LimeLightCore.LimeLightConfig
import org.psilynx.psikit.core.Logger
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

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
            kP = 0.004,
            kD = 0.00004,
            kI = 0.018,
            stabilityThreshold = 50.0
        )
        @JvmField var robotAngularVelkP = 0.15
        @JvmField var targetRpmTolerance = 50

        @JvmField var kS = 1.4
        @JvmField var kV = 0.0029

        @JvmField
        var controllerTurret = PIDController(
            kP = 0.001,
            kD = 0.000027,
            kI = 0.00125,
            stabilityThreshold = 0.2
        )
        @JvmField var ticksPerRev = 8192.0 * (108.0/22.0)

        @JvmField var targetPosTolerance = 50
        @JvmField var minTurretPosition = -14000.0
        @JvmField var maxTurretPosition = 14000.0
        @JvmField var limitTolerence = 50
        @JvmField var gearRatio = 23.0 / 30.0
        @JvmField var rpmFar = 3200.0
        @JvmField var rpmClose = 2975.0
        @JvmField var rpmRest = 1000.0
        @JvmField var shootClosePos = 5500.0
        @JvmField var shootFarPos = 7200.0
        // in dreapta creste pozitia
    }

    val rpm get() = encoderOuttake.getPositionAndVelocity().velocity / 28.0 * 60 * ShooterConfig.gearRatio

    var rpmFar by ShooterConfig::rpmFar
    var rpmClose by ShooterConfig::rpmClose
    var rpmRest by ShooterConfig::rpmRest

    var shootClosePos by ShooterConfig::shootClosePos
    var shootFarPos by ShooterConfig::shootFarPos

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

    private var offset = encoderTurret.getPositionAndVelocity().position

    val turretPosition get() = encoderTurret.getPositionAndVelocity().position - offset

    val turretAngle = (turretPosition / ShooterConfig.ticksPerRev + 360.0).deg

    var targetPos = 0.0

    fun tickToDeg(ticks : Double) : Double {
        return (360 / ShooterConfig.ticksPerRev) * ticks
    }

    fun degToTick(deg : Double) : Double {
        return (ShooterConfig.ticksPerRev / 360.0) * deg
    }

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
    
    fun updateTurretPosition(deltaTime: Duration) {
        powerTurret = ShooterConfig.controllerTurret.calculate(turretPosition, targetPos, deltaTime)
    }

    private fun computeHeadingPower(dt: Duration, error: Double, robotAngularVelocity: AngularVelocity = 0.radsec): Double {
        var raw = LimeLightConfig.controller.calculate(0.0, error, dt) +
                robotAngularVelocity.asRadSec * ShooterConfig.robotAngularVelkP

        if(abs(raw) < 0.05)
            raw = 0.0

        if (turretPosition >= ShooterConfig.maxTurretPosition /*&& error > 0*/ ) {
            raw = min(raw, 0.0)
        } else if (turretPosition <= ShooterConfig.minTurretPosition /*&& error < 0*/ ){
            raw = max(raw, 0.0)
        }

        return raw.coerceIn(-1.0, 1.0)
    }

    fun updateTurret (deltaTime: Duration, error: Double, robotAngularVelocity: AngularVelocity = 0.radsec) {
        powerTurret = computeHeadingPower(deltaTime, error, robotAngularVelocity)
    }

    fun addTelemetry(telemetry: Telemetry) {
        Logger.recordOutput("Shooter/Outtake power", powerShooter)
        Logger.recordOutput("Shooter/Outtake rpm", rpm)
        Logger.recordOutput("Shooter/Turret power", powerTurret)
        Logger.recordOutput("Shooter/Turret pos", turretPosition)
        telemetry.addData("Outtake power", powerShooter)
        telemetry.addData("Outtake rpm", rpm)
        telemetry.addData("Turret power", powerTurret)
        telemetry.addData("Turret pos", turretPosition)
    }
}