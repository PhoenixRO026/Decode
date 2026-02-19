package org.firstinspires.ftc.teamcode.prepPositions

import com.acmerobotics.roadrunner.now
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp
class TrasferPositions : LinearOpMode() {
    override fun runOpMode() {
        val servo1 = hardwareMap.get(Servo::class.java, "servoTransferBack")
        val servo2 = hardwareMap.get(Servo::class.java, "servoTransferFront")

        var previousTime: Double
        var deltaTime : Double
        var now : Double

        waitForStart()

        previousTime = now()
        servo1.position = 0.0
        servo2.position = 0.0

        while (opModeIsActive()){
            now = now()
            deltaTime = now - previousTime
            previousTime = now

            if (gamepad1.dpad_right) {
                servo1.position += 0.1 * deltaTime
                servo2.position += 0.1 * deltaTime
            }
            if (gamepad1.dpad_left) {
                servo1.position -= 0.1 * deltaTime
                servo2.position -= 0.1 * deltaTime
            }

            telemetry.addData("deltaTime", deltaTime)
            telemetry.addData("pos", servo1.position)
            telemetry.addData("dpad right", gamepad1.dpad_right)
            telemetry.addData("dpad left", gamepad1.dpad_left)
            telemetry.update()
        }
    }
}