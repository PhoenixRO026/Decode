package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.commonlibs.units.Angle
import com.commonlibs.units.Duration
import com.commonlibs.units.deg
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import kotlin.math.absoluteValue
import kotlin.math.roundToInt

class Spindexer(
    val motor: DcMotorEx,
    val finger: Servo,
    val storage: BallStorage = BallStorage()
) {
    @Config
    data object SpindexerConfig {
        @JvmField
        var pidController = PIDController(
            kP = 0.0,
            kI = 0.0,
            kD = 0.0
        )
        @JvmField
        var fingerUpPosition = 0.5
        @JvmField
        var fingerDownPosition = 0.95
        @JvmField
        var TICKS_PER_REV = ((((1.0+(46.0/17.0))) * (1.0+(46.0/11.0))) * 28.0)
        @JvmField
        var shooterOffsetDeg = 20.0
    }

    constructor(hardwareMap: HardwareMap) : this(
        motor = hardwareMap.get(DcMotorEx::class.java, "motorTransfer"),
        finger = hardwareMap.get(Servo::class.java, "finger")
    ) {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.direction = DcMotorSimple.Direction.REVERSE
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    enum class Position(val angle: Angle, val index: Int) {
        INTAKE_1(0.deg, 0),
        INTAKE_2(120.deg, 1),
        INTAKE_3(240.deg, 2),
        SHOOTER_1(SpindexerConfig.shooterOffsetDeg.deg + 120.deg, 0),
        SHOOTER_2(SpindexerConfig.shooterOffsetDeg.deg + 240.deg, 1),
        SHOOTER_3(SpindexerConfig.shooterOffsetDeg.deg, 2),
        CUSTOM(0.deg, 0)
    }

    val positionTicks get() = motor.currentPosition - ticksOffset

    private var ticksOffset = motor.currentPosition

    val positionDegrees get() =
        ((positionTicks / SpindexerConfig.TICKS_PER_REV * 360.0) % 360.0).deg

    private var currentPosition = Position.INTAKE_1

    private var _targetTicks = positionTicks

    var targetTicks
        get() = _targetTicks
        set(value) {
            _targetTicks = value
            currentPosition = Position.CUSTOM
        }

    private var _targetDegrees = positionDegrees
        set(value) {
            val targetDegs = value
            val errorDegs = targetDegs - positionDegrees
            val correctionDegs = if (errorDegs.asDeg.absoluteValue > 180) {
                180.deg - errorDegs
            } else errorDegs
            val spins = positionTicks % SpindexerConfig.TICKS_PER_REV +
                    if (correctionDegs.asDeg > 360) 1 else
                        if (correctionDegs.asDeg < 0) -1 else 0
            _targetTicks = ((spins + targetDegs.asRev) * SpindexerConfig.TICKS_PER_REV)
                .roundToInt()
            field = value
        }

    var targetDegrees
        get() = _targetDegrees
        set(value) {
            _targetDegrees = value
            currentPosition = Position.CUSTOM
        }


    var targetPosition
        get() = currentPosition
        set(value) {
            _targetDegrees = value.angle
            currentPosition = value
        }

    var fingerPos by finger::position

    var _power by motor::power

    val power get() = _power

    private fun getClosestIntakePos(currentAngle: Angle = positionDegrees): Position {
        return when {
            currentAngle.diffTo(Position.INTAKE_2.angle).asDeg.absoluteValue <= 60 -> Position.INTAKE_2
            currentAngle.diffTo(Position.INTAKE_3.angle).asDeg.absoluteValue <= 60 -> Position.INTAKE_3
            else -> Position.INTAKE_1
        }
    }

    private fun getClosestShooterPos(currentAngle: Angle = positionDegrees): Position {
        return when {
            currentAngle.diffTo(Position.SHOOTER_2.angle).asDeg.absoluteValue <= 60 -> Position.SHOOTER_2
            currentAngle.diffTo(Position.SHOOTER_3.angle).asDeg.absoluteValue <= 60 -> Position.SHOOTER_3
            else -> Position.SHOOTER_1
        }
    }

    fun greenToShooter() {
        if (!storage.hasGreen()) return
        targetPosition = storage.getClosestGreenShooterPosition(positionDegrees)
    }

    fun purpleToShooter() {
        if (!storage.hasPurple()) return
        targetPosition = storage.getClosestPurpleShooterPosition(positionDegrees)
    }

    fun fingerUp() {
        fingerPos = SpindexerConfig.fingerUpPosition
    }
    fun fingerDown() {
        fingerPos = SpindexerConfig.fingerDownPosition
    }

    fun update(deltaTime: Duration) {
        _power = SpindexerConfig.pidController
            .calculate(positionTicks.toDouble(), targetTicks.toDouble(), deltaTime)
    }

    fun resetPos() {
        ticksOffset = motor.currentPosition
        _targetTicks = 0
        currentPosition = Position.INTAKE_1
    }

    fun init() {
        fingerDown()
    }

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addLine("==== Spindexer ====")
        telemetry.addData("current pos deg", positionDegrees)
        telemetry.addData("target pos deg", targetDegrees)
        telemetry.addData("current pos state", currentPosition)
        telemetry.addData("current pos ticks", positionTicks)
        telemetry.addData("target pos ticks", targetTicks)
        telemetry.addData("power", power)
        storage.addTelemetry(telemetry)
    }
}