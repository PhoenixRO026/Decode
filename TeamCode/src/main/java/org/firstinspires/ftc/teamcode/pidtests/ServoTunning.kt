package org.firstinspires.ftc.teamcode.pidtests

import com.acmerobotics.roadrunner.now
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.library.TimeKeep

@TeleOp
class ServoTunning: LinearOpMode() {
    override fun runOpMode() {
        val servoFront = hardwareMap.get(Servo::class.java, "servoTransferFront")
        val servoBack = hardwareMap.get(Servo::class.java, "servoTransferBack")

        val timeKeep = TimeKeep()
        var previousTime: Double
        var deltaTime : Double
        var now : Double

        servoFront.position = 0.0067
        servoBack.position = 0.00

        waitForStart()
        previousTime = now()

        while (opModeIsActive()) {
            now = now()
            deltaTime = now - previousTime
            previousTime = now
            val increment = (deltaTime * 0.1)
            if(gamepad1.dpad_up) {
                servoFront.position = (servoFront.position + increment).coerceIn(0.0, 1.0 - 0.0044)
            }
            if(gamepad1.dpad_down) {
                servoFront.position = (servoFront.position - increment).coerceIn(0.0, 1.0 - 0.0044)
            }
            servoBack.position = servoFront.position - 0.0067

            telemetry.addData("pos ", servoBack.position)
            telemetry.update()
        }

    }
}