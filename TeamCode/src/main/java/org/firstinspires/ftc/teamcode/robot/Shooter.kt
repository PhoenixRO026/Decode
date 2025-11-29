package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.RaceAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.Encoder
import com.commonlibs.units.Duration
import com.commonlibs.units.SleepAction
import com.commonlibs.units.s
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.library.controller.PIDController
import kotlin.math.abs

class Shooter(
    val motorTop: DcMotorEx,
    val motorBottom: DcMotorEx,
    val encoder: Encoder
)
{
    @Config
    data object ShooterConfig {
        @JvmField
        var controller = PIDController(
            kP = 0.015,
            kD = 0.00069420,
            kI = 0.0001,
            stabilityThreshold = 0.2
        )
        @JvmField var targetRpmTolerance = 50
    }
    enum class Mode {
        PID,
        RAW_POWER
    }

    private var currentMode = Mode.RAW_POWER

    private var rpm = 0

    private var offset = 0.0

    var outtakeTargetRpm = rpm
        set(value) {
            field = value
            currentMode = Mode.PID
        }

    private var _power
        get() = motorTop.power
        set(value) {
            motorTop.power = value
            motorBottom.power = value
        }


    var power
        get() = _power
        set(value) {
            if (currentMode != Mode.RAW_POWER && value == 0.0) return
            if (rpm < 0 ) {
                rpm = abs(rpm)
                return
            }
            _power = value
            currentMode = Mode.RAW_POWER
        }

    fun update(deltaTime: Duration) {
        _power = ShooterConfig.controller.calculate(rpm.toDouble(), outtakeTargetRpm.toDouble(), deltaTime)
    }


    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("Outtake power", power)

        //telemetry.addData("lift current", rightMotor.getCurrent(CurrentUnit.AMPS) + leftMotor.getCurrent(CurrentUnit.AMPS))
    }
}