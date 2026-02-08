package org.firstinspires.ftc.teamcode.robot

import android.graphics.Color
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.RaceAction
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.SleepAction
import com.commonlibs.units.SleepAction
import com.commonlibs.units.s
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import org.firstinspires.ftc.robotcore.external.Telemetry

class Intake(
    val motor: DcMotorEx,
)
{
    var power
        get() = motor.power
        set(value) {
            motor.power = value
        }

    fun startIntakeAction() = InstantAction { power = 0.75 }
    fun stopIntakeAction() = InstantAction { power = 0.0 }

    fun spew() = SequentialAction (
        InstantAction{ power = -1.0 },
        SleepAction(0.5.s),
        stopIntakeAction()
    )

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("Intake power", power)

        //telemetry.addData("lift current", rightMotor.getCurrent(CurrentUnit.AMPS) + leftMotor.getCurrent(CurrentUnit.AMPS))
    }
}