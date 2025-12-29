package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.commonlibs.units.Angle
import com.commonlibs.units.Duration
import com.commonlibs.units.deg
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import kotlin.math.absoluteValue
import kotlin.math.roundToInt

class Spindexer(
    val motor: DcMotorEx,
    val finger: Servo
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
        var SHOOTER_OFFSET_DEG = 20.0
    }

    enum class Position(val deg: Angle) {
        INTAKE_1(0.deg),
        INTAKE_2(120.deg),
        INTAKE_3(240.deg),
        SHOOTER_1(SpindexerConfig.SHOOTER_OFFSET_DEG.deg),
        SHOOTER_2(SpindexerConfig.SHOOTER_OFFSET_DEG.deg + 120.deg),
        SHOOTER_3(SpindexerConfig.SHOOTER_OFFSET_DEG.deg + 240.deg),
        CUSTOM(0.deg)
    }

    fun getClosestIntakePos(): Position {
        return when(positionDegrees.asDeg) {
            in 60.0..<180.0 -> Position.INTAKE_2
            in 180.0..<300.0 -> Position.INTAKE_3
            else -> Position.INTAKE_1
        }
    }

    val positionTicks get() = motor.currentPosition

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
            _targetDegrees = value.deg
            currentPosition = value
        }

    var fingerPos by finger::position

    var _power by motor::power

    val power get() = _power

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
}