package org.firstinspires.ftc.teamcode.robot

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry

class Intake(
    val motor: DcMotorEx
) {
    constructor(hardwareMap: HardwareMap) : this(
        motor = hardwareMap.get(DcMotorEx::class.java, "motorIntake")
    ) {
        motor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        motor.direction = DcMotorSimple.Direction.FORWARD
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    var power by motor::power

    fun addTelemetry(telemetry: Telemetry) {
        telemetry.addLine("====== INTAKE ======")
        telemetry.addData("Intake power", power)
    }
}