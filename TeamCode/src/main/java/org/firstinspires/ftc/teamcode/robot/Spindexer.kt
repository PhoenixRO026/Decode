package org.firstinspires.ftc.teamcode.robot

import android.graphics.Color
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Action
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.RaceAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.Encoder
import com.commonlibs.units.Duration
import com.commonlibs.units.SleepAction
import com.commonlibs.units.s
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import kotlin.math.abs

class Spindexer(
    val motor: DcMotorEx,
    val encoder: Encoder,
    val finger: Servo,
    val colorSensor: NormalizedColorSensor
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
        val fingerDownPosition = 0.9
    }

    enum class Mode {
        PID,
        MANUAL
    }

    enum class SensorColor {
        PURPLE,
        GREEN,
        NONE
    }

    var sensorHue: Float = 0f

    var hsv = floatArrayOf(0f, 0f, 0f)

    val sensorColor get() = when {
        hsv[1] != 0f && sensorHue in 0f..40f -> SensorColor.PURPLE
        hsv[1] != 0f && sensorHue in 200f..280f -> SensorColor.GREEN
        hsv[1] != 0f && sensorHue in 40f..110f -> SensorColor.NONE
        else -> SensorColor.NONE
    }

    fun updateHue() {
        val normalizedColors = colorSensor.normalizedColors
        Color.RGBToHSV(
            (normalizedColors.red * 256).toInt(),
            (normalizedColors.green * 256).toInt(),
            (normalizedColors.blue * 256).toInt(),
            hsv
        )
        sensorHue = hsv[0]
    }

    private var currentMode = Mode.PID

    val position get() = encoder.getPositionAndVelocity().position - offset

    private var offset = encoder.getPositionAndVelocity().position

    var fingerPosition : Double = 0.5
        get() = finger.position
        set(value) {
            val clampedVal = value.coerceIn(0.0, 1.0)
            //
            // if (clampedVal == field) return
            field = clampedVal
            finger.position = field
        }

    private var _power: Double
        get() = motor.power
        set(value) {
            motor.power = value.coerceIn(-1.0, 1.0)
        }

    var power
        get() = _power
        set(value) {
            if (value == 0.0 && currentMode == Mode.PID) return
            currentMode = Mode.MANUAL
            _power = value
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
        InstantAction { fingerDown() },
        SleepAction(0.3.s)
    )
    var targetPosition : Double = position
        set(value) {
            currentMode = Mode.PID
            field = value
        }


    fun goToPos(pos: Double, multiplier: Int = 0, offset: Double= 0.0) {
        targetPosition = pos * multiplier + offset
        fingerDown()
    }

    fun goToPosAction(pos: Double, multiplier: Int = 0, offset: Double = 1.0) = object : Action {
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
        if (currentMode == Mode.PID) {
            _power = transferConfig.controller.calculate(
                position,
                targetPosition,
                deltaTime
            )
        }
    }

    fun waitForColorAction(waitColor: SensorColor, maxTime: Duration = 1.s) = RaceAction(
        Action {
            updateHue()
            it.addLine("Waiting for $waitColor")
            sensorColor != waitColor
        },
        SleepAction(maxTime)
    )

    fun waitForColors(duration: Duration) = RaceAction(
        waitForColorAction(SensorColor.PURPLE),
        waitForColorAction(SensorColor.GREEN),
        SleepAction(duration)
    )

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("transfer power", power)
        telemetry.addData("spindexer pos", motor.currentPosition)
        telemetry.addData("finger pos", fingerPosition)
        //telemetry.addData("lift current", rightMotor.getCurrent(CurrentUnit.AMPS) + leftMotor.getCurrent(CurrentUnit.AMPS))
    }

    fun resetPos() {
        offset = encoder.getPositionAndVelocity().position
    }
}