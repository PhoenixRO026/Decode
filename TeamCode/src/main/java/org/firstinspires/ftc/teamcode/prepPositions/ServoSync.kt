package org.firstinspires.ftc.teamcode.teleop.prepPositions

import com.acmerobotics.roadrunner.now
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp
class ServoSync: LinearOpMode() {

    override fun runOpMode() {

        val servoFront = hardwareMap.get(Servo::class.java, "servoTransferFront")
        val servoBack = hardwareMap.get(Servo::class.java, "servoTransferBack")

        var previousTime: Double
        var deltaTime : Double
        var now : Double


        waitForStart()
        previousTime = now()

        servoFront.position = 0.0
        servoBack.position = 0.0

        while (opModeIsActive()){
            now = now()
            deltaTime = now - previousTime
            previousTime = now

            if(gamepad1.a) {
                servoFront.position += 0.01 * deltaTime
            }
            else if(gamepad1.y) {
                servoFront.position -= 0.01 * deltaTime
            }
            if (gamepad1.x){
                servoBack.position += 0.01 * deltaTime
            }
            else if(gamepad1.b){
                servoBack.position -= 0.01 * deltaTime
            }

            telemetry.addData("a Pressed", gamepad1.a)
            telemetry.addData("y Pressed", gamepad1.y)
            telemetry.addData("x Pressed", gamepad1.x)
            telemetry.addData("b Pressed", gamepad1.b)
            telemetry.addData("ServoLeft", servoFront.position)
            telemetry.addData("ServoRight", servoBack.position)
            telemetry.addData("deltaTime", deltaTime)
            telemetry.update()
        }
    }
}