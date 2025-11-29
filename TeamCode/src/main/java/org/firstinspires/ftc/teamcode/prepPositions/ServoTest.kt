package org.firstinspires.ftc.teamcode.teleop.prepPositions

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.now
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp
class ServoTest: LinearOpMode() {

    @Config
    data object servoConfig {
        @JvmField
        var servoPos = 0.0
    }
    override fun runOpMode() {

        val servo = hardwareMap.get(Servo::class.java, "servo")

        var previousTime: Double
        var deltaTime : Double
        var now : Double


        waitForStart()
        previousTime = now()

        servo.position = 0.5

        while (opModeIsActive()){
            now = now()
            deltaTime = now - previousTime
            previousTime = now

            servo.position= servoConfig.servoPos



            telemetry.addData("a Pressed", gamepad1.a)
            telemetry.addData("y Pressed", gamepad1.y)
            telemetry.addData("x Pressed", gamepad1.x)
            telemetry.addData("b Pressed", gamepad1.b)
            telemetry.addData("Servo", servo.position)
            telemetry.addData("deltaTime", deltaTime)
            telemetry.update()
        }
    }
}