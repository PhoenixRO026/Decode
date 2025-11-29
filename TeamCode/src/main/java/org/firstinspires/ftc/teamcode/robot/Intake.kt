package org.firstinspires.ftc.teamcode.robot

import android.graphics.Color
import com.acmerobotics.roadrunner.InstantAction
import com.acmerobotics.roadrunner.ParallelAction
import com.acmerobotics.roadrunner.RaceAction
import com.acmerobotics.roadrunner.SequentialAction
import com.commonlibs.units.SleepAction
import com.commonlibs.units.s
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import org.firstinspires.ftc.robotcore.external.Telemetry

class Intake(
    val motor: DcMotorEx,
)
{

    enum class SensorColor {
        GREEN,
        PURPLE,
        OTHER
    }

    var power
        get() = motor.power
        set(value) {
            motor.power = value
        }


    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addData("Outtake power", power)

        //telemetry.addData("lift current", rightMotor.getCurrent(CurrentUnit.AMPS) + leftMotor.getCurrent(CurrentUnit.AMPS))
    }
}