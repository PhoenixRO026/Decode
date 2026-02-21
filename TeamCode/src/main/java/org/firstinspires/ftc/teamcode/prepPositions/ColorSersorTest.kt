package org.firstinspires.ftc.teamcode.prepPositions

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.robot.Spindexer

@TeleOp
class ColorSersorTest : LinearOpMode() {
    override fun runOpMode() {
        val servoTransferFront = hardwareMap.get(Servo::class.java, "servoTransferFront")
        val servoTransferBack = hardwareMap.get(Servo::class.java, "servoTransferBack")
        val finger = hardwareMap.get(Servo::class.java, "finger")

        val colorSensor = hardwareMap.get(NormalizedColorSensor::class.java, "colorSensor")

        var gain = 15f

        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        val transfer = Spindexer(
            servoTransfer1 = servoTransferFront,
            servoTransfer2 = servoTransferBack,
            finger = finger,
            colorSensor = colorSensor
        )

        waitForStart()

        while (opModeIsActive()) {


            // Update the gain value if either of the A or B gamepad buttons is being held
            if (gamepad1.a) {
                // Only increase the gain by a small amount, since this loop will occur multiple times per second.
                gain += 0.005.toFloat()
            } else if (gamepad1.b && gain > 1) { // A gain of less than 1 will make the values smaller, which is not helpful.
                gain -= 0.005.toFloat()
            }

            colorSensor.setGain(gain)

            transfer.updateHue()

            telemetry.addData("Gain", gain)
            telemetry.addData("color", transfer.sensorColor)
            telemetry.addData("hue", transfer.sensorHue)
            telemetry.addData("hsv", transfer.hsv)
            telemetry.update()

        }
    }
}