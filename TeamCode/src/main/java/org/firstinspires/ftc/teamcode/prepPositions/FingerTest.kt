package org.firstinspires.ftc.teamcode.prepPositions

import com.acmerobotics.roadrunner.now
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.library.TimeKeep

@TeleOp
class FingerTest : LinearOpMode() {
    override fun runOpMode() {
        val finger = hardwareMap.get(Servo::class.java, "finger")

        var previousTime: Double
        var deltaTime: Double
        var now: Double

        waitForStart()
        previousTime = now()

        while (opModeIsActive()) {
            deltaTime = now() - previousTime
            previousTime = now()

            if (gamepad1.dpad_up) {
                finger.position += 0.01 * deltaTime
            }
            if (gamepad1.dpad_down) {
                finger.position -= 0.01 * deltaTime
            }

        }

    }
}