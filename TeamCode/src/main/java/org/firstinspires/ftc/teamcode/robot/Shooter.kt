package org.firstinspires.ftc.teamcode.robot

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.RaceAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.ftc.Encoder
import com.commonlibs.units.Duration
import com.commonlibs.units.SleepAction
import com.commonlibs.units.rpm
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
            kP = 0.009,
            kD = 0.0035,
            kI = 0.000005,
            stabilityThreshold = 50.0
        )
        @JvmField var targetRpmTolerance = 50
    }

    var rpm = 0.0

    var targetRpm = 0.0

    var _power
        get() = motorTop.power
        set(value) {
            motorTop.power = value
            motorBottom.power = value
        }


    var power
        get() = _power
        set(value) {
            if (targetRpm == 0.0)
                _power = 0.0
            else {
                _power = value.coerceIn(-1.0, 1.0)
            }
        }

    fun goToRmp(rpm : Double) {
        targetRpm= rpm
    }

    fun update(deltaTime: Duration) {
        power = ShooterConfig.controller.calculate(rpm, targetRpm, deltaTime)
    }


    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("Outtake power", power)

        //telemetry.addData("lift current", rightMotor.getCurrent(CurrentUnit.AMPS) + leftMotor.getCurrent(CurrentUnit.AMPS))
    }
}