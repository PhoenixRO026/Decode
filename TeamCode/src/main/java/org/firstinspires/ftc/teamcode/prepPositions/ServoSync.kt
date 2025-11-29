package org.firstinspires.ftc.teamcode.teleop.prepPositions

import com.acmerobotics.roadrunner.now
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp
class ServoSync: LinearOpMode() {

    override fun runOpMode() {

        val servoLeft = hardwareMap.get(Servo::class.java, "servoGearShiftLeft")
        val servoRight = hardwareMap.get(Servo::class.java, "servoGearShiftRight")

        var previousTime: Double
        var deltaTime : Double
        var now : Double


        waitForStart()
        previousTime = now()

        servoLeft.position = 0.5
        servoRight.position = 0.5

        while (opModeIsActive()){
            now = now()
            deltaTime = now - previousTime
            previousTime = now

            if(gamepad1.a) {
                servoLeft.position += 0.1 * deltaTime
            }
            else if(gamepad1.y) {
                servoLeft.position -= 0.1 * deltaTime
            }
            if (gamepad1.x){
                servoRight.position += 0.1 * deltaTime
            }
            else if(gamepad1.b){
                servoRight.position -= 0.1 * deltaTime
            }

            telemetry.addData("a Pressed", gamepad1.a)
            telemetry.addData("y Pressed", gamepad1.y)
            telemetry.addData("x Pressed", gamepad1.x)
            telemetry.addData("b Pressed", gamepad1.b)
            telemetry.addData("ServoLeft", servoLeft.position)
            telemetry.addData("ServoRight", servoRight.position)
            telemetry.addData("deltaTime", deltaTime)
            telemetry.update()
        }
    }
}