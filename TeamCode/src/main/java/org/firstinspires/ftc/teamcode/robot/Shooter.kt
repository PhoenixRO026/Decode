package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.Encoder
import com.commonlibs.units.Angle
import com.commonlibs.units.AngularVelocity
import com.commonlibs.units.Distance2d
import com.commonlibs.units.Duration
import com.commonlibs.units.M
import com.commonlibs.units.Vector2d
import com.commonlibs.units.angle
import com.commonlibs.units.deg
import com.commonlibs.units.pose
import com.commonlibs.units.radsec
import com.commonlibs.units.rev
import com.commonlibs.units.rpm
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import org.firstinspires.ftc.teamcode.robot.LimeLightCore.LimeLightConfig
import org.psilynx.psikit.core.Logger
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

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
        @JvmField var targetRpmTolerance = 30

        @JvmField var kS = 1.4
        @JvmField var kV = 0.0029

        @JvmField
        var controllerTurret = PIDController(
            kP = 0.001,
            kD = 0.000025,
            kI = 0.001,
            stabilityThreshold = 0.2
        )
        @JvmField
        var controllerAngleHold = PIDController(
            kP = 0.03,
            kD = 0.0008,
            kI = 0.001,
            stabilityThreshold = 0.2,
        )
        @JvmField var ticksPerRev = 8192.0 * (108.0/22.0)

        @JvmField var targetPosTolerance = 75
        @JvmField var minTurretPosition = -14000.0
        @JvmField var maxTurretPosition = 14000.0
        @JvmField var limitTolerence = 50
        @JvmField var gearRatio = 23.0 / 30.0
        @JvmField var rpmFar = 3200.0
        @JvmField var rpmClose = 2900.0
        @JvmField var rpmRest = 1000.0
        @JvmField var shootClosePos = 5700.0
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

    enum class MODE {
        PID,
        MANUAL
    }
    var currentMode = MODE.PID

    private var offset = encoderTurret.getPositionAndVelocity().position

    val turretPosition get() = encoderTurret.getPositionAndVelocity().position - offset

    val turretAngle = (turretPosition / ShooterConfig.ticksPerRev * 360.0).deg

    var targetPos = 0.0
    var turretTargetAngle = (targetPos / ShooterConfig.ticksPerRev * 360.0).deg

    fun tickToDeg(ticks : Double) : Double {
        return (360 / ShooterConfig.ticksPerRev) * ticks
    }

    fun degToTick(deg : Double) : Double {
        return (ShooterConfig.ticksPerRev / 360.0) * deg
    }

    fun goToRmp(rpm : Double) {
        targetRpm = rpm
    }

    fun shooterBusy(): Boolean {
        val busy = abs(targetRpm - rpm) > ShooterConfig.targetRpmTolerance
        Logger.recordOutput("Shooter/shooterBusy", busy)
        return busy
    }

    fun goToRpmAction(rpm: Double) = object : Action {
        var init = true
        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                goToRmp(rpm)
            }
            return shooterBusy()
        }
    }

    fun updateRpm(deltaTime: Duration) {
        val voltage = voltageSensor.voltage

        val pidPower = ShooterConfig.controllerRpm.calculate(rpm, targetRpm, deltaTime)

        val feedforwardPower = ShooterConfig.kS + ShooterConfig.kV * targetRpm

        powerShooter = pidPower + feedforwardPower / voltage

        Logger.recordOutput("Shooter/currentRPM", rpm.rpm)
        Logger.recordOutput("Shooter/targetRPM", targetRpm.rpm)
    }

    fun goToPos(pos: Double) {
        targetPos = pos
    }

    fun turretBusy(): Boolean {
        val busy = abs(targetPos - turretPosition) > ShooterConfig.targetPosTolerance
        Logger.recordOutput("Shooter/turretBusy", busy)
        return busy
    }

    fun turretToPosAction(pos: Double) = object : Action {
        var init = true
        override fun run(p: TelemetryPacket): Boolean {
            if (init) {
                init = false
                targetPos = pos
            }
            p.addLine("waiting for turret")
            return turretBusy()
        }
    }

    fun stopShootAction() = ParallelAction(
        InstantAction { goToRpmAction(0.0)}
    )
    
    fun updateTurretPosition(deltaTime: Duration) {
        powerTurret = ShooterConfig.controllerTurret.calculate(turretPosition, targetPos, deltaTime)
        Logger.recordOutput("Shooter/currentAngle", turretAngle)
        Logger.recordOutput("Shooter/targetAngle", turretTargetAngle)
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

    private var targetAngle = 0.0

    fun resetTargetAngle(robotAngle: Angle) {
        targetAngle = tickToDeg(turretPosition) - robotAngle.asDeg
    }

    fun updateTurret (deltaTime: Duration, error: Double, robotAngularVelocity: AngularVelocity = 0.radsec, robotAngle: Angle = 0.deg) {
        if (currentMode != MODE.PID) {
            return
        }
      powerTurret = computeHeadingPower(deltaTime, error, robotAngularVelocity)
//        powerTurret = turretAngleHold(deltaTime, error, robotAngle.asDeg)
    }

//    var errorCache = 0.0

    fun turretAngleHold(dt: Duration, errorDeg: Double, robotAngleDeg: Double): Double {
        val turretAngle = tickToDeg(turretPosition) - robotAngleDeg
        if (errorDeg != 0.0) {
            targetAngle = turretAngle + errorDeg
        }
//        val errorDegGood = if (errorDeg != 0.0) {
//            errorCache = errorDeg
//            errorDeg
//        } else {
//            errorCache
//        }
//        val turretError = if (errorDeg != 0.0) turretAngle - targetAngle else 0.0
        return ShooterConfig.controllerAngleHold.calculate(turretAngle , targetAngle, dt)
    }

    private fun normalizeDegrees(a: Angle): Angle {
        var d = a.asDeg % 360.0

        if (abs(d) > 180.0){
            d = -(d - 180 * d.sign)
        }

        return d.deg
    }

    fun aimTowardsTargetPose(robotPose: Pose2d, target: Distance2d) { // robot pose should be (robot.position, turretHeading)
        val desiredFieldPose = Vector2d(robotPose.pose.position.x, robotPose.pose.position.y).headingTowards(target)

        val targetAngle = normalizeDegrees(desiredFieldPose.heading)

        targetPos = degToTick(targetAngle.asDeg).coerceIn(ShooterConfig.minTurretPosition, ShooterConfig.maxTurretPosition)
    }

    fun addTelemetry(telemetry: Telemetry) {
        Logger.recordOutput("Shooter/Outtake power", powerShooter)
//        Logger.recordOutput("Shooter/Outtake rpm", rpm)
        Logger.recordOutput("Shooter/Turret power", powerTurret)
//        Logger.recordOutput("Shooter/Turret pos", turretPosition)
        telemetry.addData("Outtake power", powerShooter)
        telemetry.addData("Outtake rpm", rpm)
        telemetry.addData("Turret power", powerTurret)
        telemetry.addData("Turret pos", turretPosition)
    }
}