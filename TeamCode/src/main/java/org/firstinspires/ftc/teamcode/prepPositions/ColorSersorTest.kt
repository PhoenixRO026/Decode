package org.firstinspires.ftc.teamcode.prepPositions

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.robot.Spindexer

@TeleOp
class ColorSersorTest : LinearOpMode() {
    override fun runOpMode() {
        val servoTransferFront = hardwareMap.get(Servo::class.java, "servoTransferFront")
        val servoTransferBack = hardwareMap.get(Servo::class.java, "servoTransferBack")
        val finger = hardwareMap.get(Servo::class.java, "finger")

        val colorSensor = hardwareMap.get(NormalizedColorSensor::class.java, "colorSensor")

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val transfer = Spindexer(
            servoTransfer1 = servoTransferFront,
            servoTransfer2 = servoTransferBack,
            finger = finger,
            colorSensor = colorSensor
        )

        waitForStart()

        while (opModeIsActive()) {
            transfer.updateHue()
            telemetry.addData("color", transfer.sensorColor)
            telemetry.addData("hue", transfer.sensorHue)
            telemetry.addData("hsv", transfer.hsv)
            telemetry.update()

        }
    }
}