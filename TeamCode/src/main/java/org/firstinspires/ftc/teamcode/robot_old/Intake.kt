package org.firstinspires.ftc.teamcode.robot_old

import com.qualcomm.robotcore.hardware.DcMotorEx
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