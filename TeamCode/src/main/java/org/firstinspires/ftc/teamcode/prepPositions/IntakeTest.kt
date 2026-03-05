package org.firstinspires.ftc.teamcode.prepPositions

import com.acmerobotics.roadrunner.now
import com.commonlibs.units.Pose
import com.commonlibs.units.cm
import com.commonlibs.units.deg
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.library.TimeKeep
import org.firstinspires.ftc.teamcode.library.buttons.ButtonReader
import org.firstinspires.ftc.teamcode.robot.Robot
import org.firstinspires.ftc.teamcode.robot.Spindexer

@TeleOp
class IntakeTest : LinearOpMode() {
    override fun runOpMode() {
        val intakeMotor = hardwareMap.get(DcMotorEx::class.java, "motorIntake")
        val servo1 = hardwareMap.get(Servo::class.java, "servoTransferFront")
        val servo2 = hardwareMap.get(Servo::class.java, "servoTransferBack")

        servo1.position = 0.0
        servo2.position = 0.0
        val timeKeep = TimeKeep()

        val intakeStart = ButtonReader { gamepad1.right_bumper }
        val intakeStop = ButtonReader { gamepad1.left_bumper }
        val buttons = listOf(intakeStart, intakeStop)

        var previousTime: Double
        var deltaTime : Double
        var now : Double

        waitForStart()
        previousTime = now()

        while (opModeIsActive()) {
            timeKeep.resetDeltaTime()
            buttons.forEach { it.readValue() }

            now = now()
            deltaTime = now - previousTime
            previousTime = now

            if (intakeStart.wasJustPressed()) {
                intakeMotor.power = 1.0
            }
            if (intakeStop.wasJustPressed()) {
                intakeMotor.power = 0.0
            }

            if (gamepad1.dpad_right) {
                servo1.position += 0.7 * deltaTime
                servo2.position += 0.7 * deltaTime
            }
            if (gamepad1.dpad_left) {
                servo1.position -= 0.7 * deltaTime
                servo2.position -= 0.7 * deltaTime
            }
            telemetry.addData("servo position", servo1.position)
            telemetry.addData("motor power", intakeMotor.power)
            telemetry.update()
        }
    }
}