package org.firstinspires.ftc.teamcode.pidtests

import com.acmerobotics.roadrunner.now
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.library.TimeKeep

@TeleOp
class ServoTunning: LinearOpMode() {
    override fun runOpMode() {
        val servoFront = hardwareMap.get(Servo::class.java, "servoFront")
        val servoBack = hardwareMap.get(Servo::class.java, "servoBack")

        val timeKeep = TimeKeep()
        var previousTime: Double
        var deltaTime : Double
        var now : Double

        servoFront.position = 0.5
        servoBack.position = 0.5

        waitForStart()
        previousTime = now()

        while (opModeIsActive()) {
            now = now()
            deltaTime = now - previousTime
            previousTime = now
            if(gamepad1.dpad_up) {
                servoFront.position += deltaTime * 0.1
            }
            if(gamepad1.dpad_down) {
                servoFront.position -= deltaTime * 0.1
            }
            if(gamepad1.dpad_right) {
                servoBack.position += deltaTime * 0.1
            }
            if(gamepad1.dpad_left) {
                servoBack.position -= deltaTime * 0.1
            }

        }

    }
}